#include <stdio.h>
#include "pico/stdlib.h"
#include "btstack_audio.h"
#include "btstack_resample.h"
#include "btstack_sample_rate_compensation.h"
#include "btstack_ring_buffer.h"
#include "btstack.h"

#define NUM_CHANNELS 2
#define BYTES_PER_FRAME     (2*NUM_CHANNELS)
#define MAX_SBC_FRAME_SIZE 120

static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t  sdp_avdtp_sink_service_buffer[150];
static uint8_t  device_id_sdp_service_buffer[100];

// we support all configurations with bitpool 2-53
static uint8_t media_sbc_codec_capabilities[] = {
    0xFF,//(AVDTP_SBC_44100 << 4) | AVDTP_SBC_STEREO,
    0xFF,//(AVDTP_SBC_BLOCK_LENGTH_16 << 4) | (AVDTP_SBC_SUBBANDS_8 << 2) | AVDTP_SBC_ALLOCATION_METHOD_LOUDNESS,
    2, 53
};


static btstack_sbc_decoder_state_t state;
static btstack_sbc_mode_t mode = SBC_MODE_STANDARD;

// ring buffer for SBC Frames
// below 30: add samples, 30-40: fine, above 40: drop samples
#define OPTIMAL_FRAMES_MIN 60
#define OPTIMAL_FRAMES_MAX 80
#define ADDITIONAL_FRAMES  30
static uint8_t sbc_frame_storage[(OPTIMAL_FRAMES_MAX + ADDITIONAL_FRAMES) * MAX_SBC_FRAME_SIZE];
static btstack_ring_buffer_t sbc_frame_ring_buffer;
static unsigned int sbc_frame_size;

// overflow buffer for not fully used sbc frames, with additional frames for resampling
static uint8_t decoded_audio_storage[(128+16) * BYTES_PER_FRAME];
static btstack_ring_buffer_t decoded_audio_ring_buffer;

static int media_initialized = 0;
static int audio_stream_started;
static btstack_resample_t resample_instance;

// temp storage of lower-layer request for audio samples
static int16_t * request_buffer;
static int       request_frames;

typedef struct {
    uint8_t  reconfigure;
    uint8_t  num_channels;
    uint16_t sampling_frequency;
    uint8_t  block_length;
    uint8_t  subbands;
    uint8_t  min_bitpool_value;
    uint8_t  max_bitpool_value;
    btstack_sbc_channel_mode_t      channel_mode;
    btstack_sbc_allocation_method_t allocation_method;
} media_codec_configuration_sbc_t;

typedef enum {
    STREAM_STATE_CLOSED,
    STREAM_STATE_OPEN,
    STREAM_STATE_PLAYING,
    STREAM_STATE_PAUSED,
} stream_state_t;

typedef struct {
    uint8_t  a2dp_local_seid;
    uint8_t  media_sbc_codec_configuration[4];
} a2dp_sink_demo_stream_endpoint_t;
static a2dp_sink_demo_stream_endpoint_t a2dp_sink_demo_stream_endpoint;

typedef struct {
    bd_addr_t addr;
    uint16_t  a2dp_cid;
    uint8_t   a2dp_local_seid;
    stream_state_t stream_state;
    media_codec_configuration_sbc_t sbc_configuration;
} a2dp_sink_demo_a2dp_connection_t;
static a2dp_sink_demo_a2dp_connection_t a2dp_sink_demo_a2dp_connection;

/* @section Handle Media Data Packet 
 *
 * @text Here the audio data, are received through the handle_l2cap_media_data_packet callback.
 * Currently, only the SBC media codec is supported. Hence, the media data consists of the media packet header and the SBC packet.
 * The SBC frame will be stored in a ring buffer for later processing (instead of decoding it to PCM right away which would require a much larger buffer).
 * If the audio stream wasn't started already and there are enough SBC frames in the ring buffer, start playback.
 */ 

static int read_media_data_header(uint8_t * packet, int size, int * offset, avdtp_media_packet_header_t * media_header);
static int read_sbc_header(uint8_t * packet, int size, int * offset, avdtp_sbc_codec_header_t * sbc_header);

// Handles pairing & connecting
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

// Handles a2dp info
static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t event_size);

// Actual audio data
static void handle_l2cap_media_data_packet(uint8_t seid, uint8_t *packet, uint16_t size);
static int media_processing_init(media_codec_configuration_sbc_t * configuration);
static void media_processing_start(void);
static void media_processing_pause(void);
static void media_processing_close(void);


static void dump_sbc_configuration(media_codec_configuration_sbc_t * configuration){
    printf("    - num_channels: %d\n", configuration->num_channels);
    printf("    - sampling_frequency: %d\n", configuration->sampling_frequency);
    printf("    - channel_mode: %d\n", configuration->channel_mode);
    printf("    - block_length: %d\n", configuration->block_length);
    printf("    - subbands: %d\n", configuration->subbands);
    printf("    - allocation_method: %d\n", configuration->allocation_method);
    printf("    - bitpool_value [%d, %d] \n", configuration->min_bitpool_value, configuration->max_bitpool_value);
    printf("\n");
}

/* LISTING_END */
#ifdef HAVE_BTSTACK_AUDIO_EFFECTIVE_SAMPLERATE
btstack_sample_rate_compensation_t sample_rate_compensation;
#endif

static void playback_handler(int16_t * buffer, uint16_t num_audio_frames){

#ifdef STORE_TO_WAV_FILE
    int       wav_samples = num_audio_frames * NUM_CHANNELS;
    int16_t * wav_buffer  = buffer;
#endif
    
    // called from lower-layer but guaranteed to be on main thread
    if (sbc_frame_size == 0){
        memset(buffer, 0, num_audio_frames * BYTES_PER_FRAME);
        return;
    }

    // first fill from resampled audio
    uint32_t bytes_read;
    btstack_ring_buffer_read(&decoded_audio_ring_buffer, (uint8_t *) buffer, num_audio_frames * BYTES_PER_FRAME, &bytes_read);
    buffer          += bytes_read / NUM_CHANNELS;
    num_audio_frames   -= bytes_read / BYTES_PER_FRAME;

    // then start decoding sbc frames using request_* globals
    request_buffer = buffer;
    request_frames = num_audio_frames;
    while (request_frames && btstack_ring_buffer_bytes_available(&sbc_frame_ring_buffer) >= sbc_frame_size){
        // decode frame
        uint8_t sbc_frame[MAX_SBC_FRAME_SIZE];
        btstack_ring_buffer_read(&sbc_frame_ring_buffer, sbc_frame, sbc_frame_size, &bytes_read);
        btstack_sbc_decoder_process_data(&state, 0, sbc_frame, sbc_frame_size);
    }

#ifdef STORE_TO_WAV_FILE
    audio_frame_count += num_audio_frames;
    wav_writer_write_int16(wav_samples, wav_buffer);
#endif
}

static void handle_pcm_data(int16_t * data, int num_audio_frames, int num_channels, int sample_rate, void * context){
    UNUSED(sample_rate);
    UNUSED(context);
    UNUSED(num_channels);   // must be stereo == 2

    const btstack_audio_sink_t * audio_sink = btstack_audio_sink_get_instance();
    if (!audio_sink){
#ifdef STORE_TO_WAV_FILE
        audio_frame_count += num_audio_frames;
        wav_writer_write_int16(num_audio_frames * NUM_CHANNELS, data);
#endif
        return;
    }

    // resample into request buffer - add some additional space for resampling
    int16_t  output_buffer[(128+16) * NUM_CHANNELS]; // 16 * 8 * 2
    uint32_t resampled_frames = btstack_resample_block(&resample_instance, data, num_audio_frames, output_buffer);

    // store data in btstack_audio buffer first
    int frames_to_copy = btstack_min(resampled_frames, request_frames);
    memcpy(request_buffer, output_buffer, frames_to_copy * BYTES_PER_FRAME);
    request_frames  -= frames_to_copy;
    request_buffer  += frames_to_copy * NUM_CHANNELS;

    // and rest in ring buffer
    int frames_to_store = resampled_frames - frames_to_copy;
    if (frames_to_store){
        int status = btstack_ring_buffer_write(&decoded_audio_ring_buffer, (uint8_t *)&output_buffer[frames_to_copy * NUM_CHANNELS], frames_to_store * BYTES_PER_FRAME);
        if (status){
            printf("Error storing samples in PCM ring buffer!!!\n");
        }
    }
}

void bt_init() {
    l2cap_init();

#ifdef ENABLE_BLE
    // Initialize LE Security Manager. Needed for cross-transport key derivation
    sm_init();
#endif

    // Initialize AVDTP Sink
    a2dp_sink_init();
    a2dp_sink_register_packet_handler(&a2dp_sink_packet_handler);
    a2dp_sink_register_media_handler(&handle_l2cap_media_data_packet);

    // Create stream endpoint
    a2dp_sink_demo_stream_endpoint_t * stream_endpoint = &a2dp_sink_demo_stream_endpoint;
    avdtp_stream_endpoint_t * local_stream_endpoint = a2dp_sink_create_stream_endpoint(AVDTP_AUDIO,
                                                                                       AVDTP_CODEC_SBC, media_sbc_codec_capabilities, sizeof(media_sbc_codec_capabilities),
                                                                                       stream_endpoint->media_sbc_codec_configuration, sizeof(stream_endpoint->media_sbc_codec_configuration));
    if (!local_stream_endpoint){
        printf("A2DP Sink: not enough memory to create local stream endpoint\n");
        return;
    }

    // Store stream enpoint's SEP ID, as it is used by A2DP API to identify the stream endpoint
    stream_endpoint->a2dp_local_seid = avdtp_local_seid(local_stream_endpoint);

    
    // Initialize SDP 
    sdp_init();

    // Create A2DP Sink service record and register it with SDP
    memset(sdp_avdtp_sink_service_buffer, 0, sizeof(sdp_avdtp_sink_service_buffer));
    a2dp_sink_create_sdp_record(sdp_avdtp_sink_service_buffer, 0x10001, AVDTP_SINK_FEATURE_MASK_HEADPHONE, NULL, NULL);
    sdp_register_service(sdp_avdtp_sink_service_buffer);

    // Create Device ID (PnP) service record and register it with SDP
    memset(device_id_sdp_service_buffer, 0, sizeof(device_id_sdp_service_buffer));
    device_id_create_sdp_record(device_id_sdp_service_buffer, 0x10004, DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(device_id_sdp_service_buffer);

    // Set local name with a template Bluetooth address, that will be automatically
    // replaced with an actual address once it is available, i.e. when BTstack boots
    // up and starts talking to a Bluetooth module.
    gap_set_local_name("A2DP Sink Demo 00:00:00:00:00:00");

    // allot to show up in Bluetooth inquiry
    gap_discoverable_control(1);

    // Service Class: Audio, Major Device Class: Audio, Minor: Loudspeaker
    gap_set_class_of_device(0x200414);

    // allow for role switch in general and sniff mode
    gap_set_default_link_policy_settings( LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE );

    // allow for role switch on outgoing connections - this allows A2DP Source, e.g. smartphone, to become master when we re-connect to it
    gap_set_allow_role_switch(true);

    // Register for HCI events
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    btstack_audio_sink_set_instance(btstack_audio_pico_sink_get_instance());

    if (!btstack_audio_sink_get_instance()){
        printf("No audio playback.\n");
    } else {
        printf("Audio playback supported.\n");
    }

    printf("Starting BTstack ...\n");
    hci_power_control(HCI_POWER_ON);
}


static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) == HCI_EVENT_PIN_CODE_REQUEST) {
        bd_addr_t address;
        printf("Pin code request - using '0000'\n");
        hci_event_pin_code_request_get_bd_addr(packet, address);
        gap_pin_code_response(address, "0000");
    }
}

static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t address;
    uint8_t status;

    uint8_t allocation_method;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;

    a2dp_sink_demo_a2dp_connection_t * a2dp_conn = &a2dp_sink_demo_a2dp_connection;

    switch (packet[2]){
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_OTHER_CONFIGURATION:
            printf("A2DP  Sink      : Received non SBC codec - not implemented\n");
            break;
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:{
            printf("A2DP  Sink      : Received SBC codec configuration\n");
            a2dp_conn->sbc_configuration.reconfigure = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            a2dp_conn->sbc_configuration.num_channels = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            a2dp_conn->sbc_configuration.sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            a2dp_conn->sbc_configuration.block_length = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            a2dp_conn->sbc_configuration.subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            a2dp_conn->sbc_configuration.min_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            a2dp_conn->sbc_configuration.max_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
            
            allocation_method = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);
            
            // Adapt Bluetooth spec definition to SBC Encoder expected input
            a2dp_conn->sbc_configuration.allocation_method = (btstack_sbc_allocation_method_t)(allocation_method - 1);
           
            switch (a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet)){
                case AVDTP_CHANNEL_MODE_JOINT_STEREO:
                    a2dp_conn->sbc_configuration.channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO;
                    break;
                case AVDTP_CHANNEL_MODE_STEREO:
                    a2dp_conn->sbc_configuration.channel_mode = SBC_CHANNEL_MODE_STEREO;
                    break;
                case AVDTP_CHANNEL_MODE_DUAL_CHANNEL:
                    a2dp_conn->sbc_configuration.channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL;
                    break;
                case AVDTP_CHANNEL_MODE_MONO:
                    a2dp_conn->sbc_configuration.channel_mode = SBC_CHANNEL_MODE_MONO;
                    break;
                default:
                    btstack_assert(false);
                    break;
            }
            dump_sbc_configuration(&a2dp_conn->sbc_configuration);
            break;
        }

        case A2DP_SUBEVENT_STREAM_ESTABLISHED:
            a2dp_subevent_stream_established_get_bd_addr(packet, a2dp_conn->addr);

            status = a2dp_subevent_stream_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                printf("A2DP  Sink      : Streaming connection failed, status 0x%02x\n", status);
                break;
            }

            a2dp_conn->a2dp_cid = a2dp_subevent_stream_established_get_a2dp_cid(packet);
            a2dp_conn->stream_state = STREAM_STATE_OPEN;

            printf("A2DP  Sink      : Streaming connection is established, address %s, cid 0x%02x, local seid %d\n",
                   bd_addr_to_str(address), a2dp_conn->a2dp_cid, a2dp_conn->a2dp_local_seid);
            break;
        
#ifdef ENABLE_AVDTP_ACCEPTOR_EXPLICIT_START_STREAM_CONFIRMATION
        case A2DP_SUBEVENT_START_STREAM_REQUESTED:
            printf("A2DP  Sink      : Explicit Accept to start stream, local_seid %d\n", a2dp_subevent_start_stream_requested_get_local_seid(packet));
            a2dp_sink_start_stream_accept(a2dp_cid, a2dp_local_seid);
            break;
#endif
        case A2DP_SUBEVENT_STREAM_STARTED:
            printf("A2DP  Sink      : Stream started\n");
            a2dp_conn->stream_state = STREAM_STATE_PLAYING;
            if (a2dp_conn->sbc_configuration.reconfigure){
                media_processing_close();
            }
            // prepare media processing
            media_processing_init(&a2dp_conn->sbc_configuration);
            // audio stream is started when buffer reaches minimal level
            break;
        
        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            printf("A2DP  Sink      : Stream paused\n");
            a2dp_conn->stream_state = STREAM_STATE_PAUSED;
            media_processing_pause();
            break;
        
        case A2DP_SUBEVENT_STREAM_RELEASED:
            printf("A2DP  Sink      : Stream released\n");
            a2dp_conn->stream_state = STREAM_STATE_CLOSED;
            media_processing_close();
            break;
        
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            printf("A2DP  Sink      : Signaling connection released\n");
            a2dp_conn->a2dp_cid = 0;
            media_processing_close();
            break;
        
        default:
            break;
    }
}

static int media_processing_init(media_codec_configuration_sbc_t * configuration){
    if (media_initialized) return 0;
#ifdef HAVE_BTSTACK_AUDIO_EFFECTIVE_SAMPLERATE
    btstack_sample_rate_compensation_init( &sample_rate_compensation, btstack_run_loop_get_time_ms(), configuration->sampling_frequency, FLOAT_TO_Q15(1.f) );
#endif
    btstack_sbc_decoder_init(&state, mode, handle_pcm_data, NULL);

#ifdef STORE_TO_WAV_FILE
    wav_writer_open(wav_filename, configuration->num_channels, configuration->sampling_frequency);
#endif

    btstack_ring_buffer_init(&sbc_frame_ring_buffer, sbc_frame_storage, sizeof(sbc_frame_storage));
    btstack_ring_buffer_init(&decoded_audio_ring_buffer, decoded_audio_storage, sizeof(decoded_audio_storage));
    btstack_resample_init(&resample_instance, configuration->num_channels);

    // setup audio playback
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio){
        audio->init(NUM_CHANNELS, configuration->sampling_frequency, &playback_handler);
    }

    audio_stream_started = 0;
    media_initialized = 1;
    return 0;
}

static void media_processing_start(void){
    if (!media_initialized) return;
#ifdef HAVE_BTSTACK_AUDIO_EFFECTIVE_SAMPLERATE
    btstack_sample_rate_compensation_reset( &sample_rate_compensation, btstack_run_loop_get_time_ms() );
#endif
    // setup audio playback
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio){
        audio->start_stream();
    }
    audio_stream_started = 1;
}

static void media_processing_pause(void){
    if (!media_initialized) return;
    // stop audio playback
    audio_stream_started = 0;
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio){
        audio->stop_stream();
    }
    // discard pending data
    btstack_ring_buffer_reset(&decoded_audio_ring_buffer);
    btstack_ring_buffer_reset(&sbc_frame_ring_buffer);
}

static void media_processing_close(void){
    if (!media_initialized) return;
    media_initialized = 0;
    audio_stream_started = 0;
    sbc_frame_size = 0;

#ifdef STORE_TO_WAV_FILE                 
    wav_writer_close();
    uint32_t total_frames_nr = state.good_frames_nr + state.bad_frames_nr + state.zero_frames_nr;

    printf("WAV Writer: Decoding done. Processed %u SBC frames:\n - %d good\n - %d bad\n", total_frames_nr, state.good_frames_nr, total_frames_nr - state.good_frames_nr);
    printf("WAV Writer: Wrote %u audio frames to wav file: %s\n", audio_frame_count, wav_filename);
#endif

    // stop audio playback
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio){
        printf("close stream\n");
        audio->close();
    }
}

static void handle_l2cap_media_data_packet(uint8_t seid, uint8_t *packet, uint16_t size){
    UNUSED(seid);
    int pos = 0;
     
    avdtp_media_packet_header_t media_header;
    if (!read_media_data_header(packet, size, &pos, &media_header)) return;
    
    avdtp_sbc_codec_header_t sbc_header;
    if (!read_sbc_header(packet, size, &pos, &sbc_header)) return;

    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    // process data right away if there's no audio implementation active, e.g. on posix systems to store as .wav
    if (!audio){
        btstack_sbc_decoder_process_data(&state, 0, packet+pos, size-pos);
        return;
    }


    // store sbc frame size for buffer management
    sbc_frame_size = (size-pos)/ sbc_header.num_frames;
        
    int status = btstack_ring_buffer_write(&sbc_frame_ring_buffer, packet+pos, size-pos);
    if (status != ERROR_CODE_SUCCESS){
        printf("Error storing samples in SBC ring buffer!!!\n");
    }

    // decide on audio sync drift based on number of sbc frames in queue
    int sbc_frames_in_buffer = btstack_ring_buffer_bytes_available(&sbc_frame_ring_buffer) / sbc_frame_size;
#ifdef HAVE_BTSTACK_AUDIO_EFFECTIVE_SAMPLERATE
    // update sample rate compensation
    if( audio_stream_started && (audio != NULL)) {
        uint32_t resampling_factor = btstack_sample_rate_compensation_update( &sample_rate_compensation, btstack_run_loop_get_time_ms(), sbc_header.num_frames*128, audio->get_samplerate() );
        btstack_resample_set_factor(&resample_instance, resampling_factor);
//        printf("sbc buffer level :            %d\n", btstack_ring_buffer_bytes_available(&sbc_frame_ring_buffer));
    }
#else
    uint32_t resampling_factor;

    // nominal factor (fixed-point 2^16) and compensation offset
    uint32_t nominal_factor = 0x10000;
    uint32_t compensation   = 0x00100;

    if (sbc_frames_in_buffer < OPTIMAL_FRAMES_MIN){
    	resampling_factor = nominal_factor - compensation;    // stretch samples
    } else if (sbc_frames_in_buffer <= OPTIMAL_FRAMES_MAX){
    	resampling_factor = nominal_factor;                   // nothing to do
    } else {
    	resampling_factor = nominal_factor + compensation;    // compress samples
    }

    btstack_resample_set_factor(&resample_instance, resampling_factor);
#endif
    // start stream if enough frames buffered
    if (!audio_stream_started && sbc_frames_in_buffer >= OPTIMAL_FRAMES_MIN){
        media_processing_start();
    }
}

static int read_sbc_header(uint8_t * packet, int size, int * offset, avdtp_sbc_codec_header_t * sbc_header){
    int sbc_header_len = 12; // without crc
    int pos = *offset;
    
    if (size - pos < sbc_header_len){
        printf("Not enough data to read SBC header, expected %d, received %d\n", sbc_header_len, size-pos);
        return 0;
    }

    sbc_header->fragmentation = get_bit16(packet[pos], 7);
    sbc_header->starting_packet = get_bit16(packet[pos], 6);
    sbc_header->last_packet = get_bit16(packet[pos], 5);
    sbc_header->num_frames = packet[pos] & 0x0f;
    pos++;
    *offset = pos;
    return 1;
}

static int read_media_data_header(uint8_t *packet, int size, int *offset, avdtp_media_packet_header_t *media_header){
    int media_header_len = 12; // without crc
    int pos = *offset;
    
    if (size - pos < media_header_len){
        printf("Not enough data to read media packet header, expected %d, received %d\n", media_header_len, size-pos);
        return 0;
    }

    media_header->version = packet[pos] & 0x03;
    media_header->padding = get_bit16(packet[pos],2);
    media_header->extension = get_bit16(packet[pos],3);
    media_header->csrc_count = (packet[pos] >> 4) & 0x0F;
    pos++;

    media_header->marker = get_bit16(packet[pos],0);
    media_header->payload_type  = (packet[pos] >> 1) & 0x7F;
    pos++;

    media_header->sequence_number = big_endian_read_16(packet, pos);
    pos+=2;

    media_header->timestamp = big_endian_read_32(packet, pos);
    pos+=4;

    media_header->synchronization_source = big_endian_read_32(packet, pos);
    pos+=4;
    *offset = pos;
    return 1;
}