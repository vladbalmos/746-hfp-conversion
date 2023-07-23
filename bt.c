#include <stdio.h>
#include "btstack_config.h"
#include "pico/stdlib.h"
#include "classic/btstack_sbc.h"
#include "classic/hfp.h"
#include "classic/hfp_msbc.h"
#include "btstack.h"
#include "debug.h"


// number of sco packets until 'report' on console
#define SCO_REPORT_PERIOD           100

// pre-buffer for CVSD and mSBC - also defines latency
#define SCO_MSBC_PA_PREBUFFER_MS    50

// constants
#define NUM_CHANNELS            1
#define MSBC_SAMPLE_RATE        16000
#define BYTES_PER_FRAME         2
#define MAX_NUM_MSBC_SAMPLES (16*8)

#define MSBC_PA_PREBUFFER_BYTES (SCO_MSBC_PA_PREBUFFER_MS * MSBC_SAMPLE_RATE/1000 * BYTES_PER_FRAME)

static char cmd;

static btstack_sbc_decoder_state_t decoder_state;

static int                   audio_output_paused  = 0;
static uint8_t               audio_output_ring_buffer_storage[2*MSBC_PA_PREBUFFER_BYTES];
static btstack_ring_buffer_t audio_output_ring_buffer;

uint8_t hfp_service_buffer[150];
const uint8_t   rfcomm_channel_nr = 1;
// static uint8_t codecs[] = {HFP_CODEC_CVSD, HFP_CODEC_MSBC};
static uint8_t codecs[] = {HFP_CODEC_CVSD};
static uint16_t indicators[1] = {0x01};
const char hfp_hf_service_name[] = "746 HFP";
static btstack_packet_callback_registration_t hci_event_callback_registration;
static hci_con_handle_t acl_handle = HCI_CON_HANDLE_INVALID;
static hci_con_handle_t sco_handle = HCI_CON_HANDLE_INVALID;

static bd_addr_t device_addr;

unsigned int phase;
static const int16_t sine_int16_at_16000hz[] = {
     0,   3135,   6237,   9270,  12202,  14999,  17633,  20073,  22294,  24270,
 25980,  27406,  28531,  29344,  29835,  30000,  29835,  29344,  28531,  27406,
 25980,  24270,  22294,  20073,  17633,  14999,  12202,   9270,   6237,   3135,
     0,  -3135,  -6237,  -9270, -12202, -14999, -17633, -20073, -22294, -24270,
-25980, -27406, -28531, -29344, -29835, -30000, -29835, -29344, -28531, -27406,
-25980, -24270, -22294, -20073, -17633, -14999, -12202,  -9270,  -6237,  -3135,
};

static int64_t a(alarm_id_t id, void *user_data) {
    hfp_hf_establish_audio_connection(acl_handle);
    return 0;
}

static void dump_supported_codecs(void){
    unsigned int i;
    int mSBC_skipped = 0;
    printf("Supported codecs: ");
    for (i = 0; i < sizeof(codecs); i++){
        switch(codecs[i]){
            case HFP_CODEC_CVSD:
                printf("CVSD");
                break;
            case HFP_CODEC_MSBC:
                if (hci_extended_sco_link_supported()){
                    printf(", mSBC");
                } else {
                    mSBC_skipped = 1;
                }
                break;
            default:
                btstack_assert(false);
                break;
        }
    }
    printf("\n");
    if (mSBC_skipped){
        printf("mSBC codec disabled because eSCO not supported by local controller.\n");
    }
}

static void report_status(uint8_t status, const char * message){
    if (status != ERROR_CODE_SUCCESS){
        printf("%s command failed, status 0x%02x\n", message, status);
    } else {
        printf("%s command successful\n", message);
    }
}

static void handle_pcm_data(int16_t * data, int num_samples, int num_channels, int sample_rate, void * context){
    UNUSED(context);
    UNUSED(sample_rate);
    UNUSED(data);
    UNUSED(num_samples);
    UNUSED(num_channels);

    DEBUG("Handling pcm data. Writing to output ring buffer\n");
    // samples in callback in host endianess, ready for playback
    btstack_ring_buffer_write(&audio_output_ring_buffer, (uint8_t *)data, num_samples*num_channels*2);
}

static void playback_callback(int16_t * buffer, uint16_t num_samples){
    uint32_t prebuffer_bytes = MSBC_PA_PREBUFFER_BYTES;

    // fill with silence while paused
    if (audio_output_paused){
        if (btstack_ring_buffer_bytes_available(&audio_output_ring_buffer) < prebuffer_bytes){
            memset(buffer, 0, num_samples * BYTES_PER_FRAME);
            return;
        }
        // resume playback
        audio_output_paused = 0;
    }

    // get data from ringbuffer
    uint32_t bytes_read = 0;
    btstack_ring_buffer_read(&audio_output_ring_buffer, (uint8_t *) buffer, num_samples * BYTES_PER_FRAME, &bytes_read);
    num_samples -= bytes_read / BYTES_PER_FRAME;
    buffer      += bytes_read / BYTES_PER_FRAME;

    // fill with 0 if not enough
    if (num_samples){
        memset(buffer, 0, num_samples * BYTES_PER_FRAME);
        audio_output_paused = 1;
    }
}

static void sco_receive(uint8_t * packet, uint16_t size) {
    static uint32_t packets = 0;
    static uint32_t crc_errors = 0;
    static uint32_t data_received = 0;
    static uint32_t byte_errors = 0;

    DEBUG("Sco_receive\n");
    data_received += size - 3;
    packets++;
    if (data_received > 100000){
        printf("Summary: data %07u, packets %04u, packet with crc errors %0u, byte errors %04u\n",  (unsigned int) data_received,  (unsigned int) packets, (unsigned int) crc_errors, (unsigned int) byte_errors);
        crc_errors = 0;
        byte_errors = 0;
        data_received = 0;
        packets = 0;
    }

    DEBUG("Decoding sbc data\n");
    btstack_sbc_decoder_process_data(&decoder_state, (packet[1] >> 4) & 3, packet+3, size-3);  
}


static void sco_demo_sine_wave_int16_at_16000_hz_host_endian(unsigned int num_samples, int16_t * data){
    unsigned int i;
    for (i=0; i < num_samples; i++){
        data[i] = sine_int16_at_16000hz[phase++];
        if (phase >= (sizeof(sine_int16_at_16000hz) / sizeof(int16_t))){
            phase = 0;
        }
    }
}

static void sco_demo_msbc_fill_sine_audio_frame(void){
    if (!hfp_msbc_can_encode_audio_frame_now()) return;
    int num_samples = hfp_msbc_num_audio_samples_per_frame();
    if (num_samples > MAX_NUM_MSBC_SAMPLES) return;
    int16_t sample_buffer[MAX_NUM_MSBC_SAMPLES];
    sco_demo_sine_wave_int16_at_16000_hz_host_endian(num_samples, sample_buffer);
    hfp_msbc_encode_audio_frame(sample_buffer);
}

static void sco_send(hci_con_handle_t sco_handle) {
    if (sco_handle == HCI_CON_HANDLE_INVALID) return;
    
    int sco_packet_length = hci_get_sco_packet_length();
    int sco_payload_length = sco_packet_length - 3;

    hci_reserve_packet_buffer();
    uint8_t * sco_packet = hci_get_outgoing_packet_buffer();

    if (hfp_msbc_num_bytes_in_stream() < sco_payload_length){
        log_error("mSBC stream is empty.");
    }
    hfp_msbc_read_from_stream(sco_packet + 3, sco_payload_length);
    sco_demo_msbc_fill_sine_audio_frame();
}

static void sco_init_codec() {
    DEBUG("Initializing codec\n");
    btstack_sbc_decoder_init(&decoder_state, SBC_MODE_mSBC, &handle_pcm_data, NULL);
    hfp_msbc_init();

    memset(audio_output_ring_buffer_storage, 0, sizeof(audio_output_ring_buffer_storage));
    btstack_ring_buffer_init(&audio_output_ring_buffer, audio_output_ring_buffer_storage, sizeof(audio_output_ring_buffer_storage));

    // config and setup audio playback
    const btstack_audio_sink_t * audio_sink = btstack_audio_sink_get_instance();

    audio_sink->init(1, MSBC_SAMPLE_RATE, &playback_callback);
    audio_sink->start_stream();

    audio_output_paused  = 1;
    DEBUG("Codec initialized\n");
}


static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * event, uint16_t event_size){
    UNUSED(channel);
    uint8_t status;
    bd_addr_t event_addr;

    switch (packet_type){

        case HCI_SCO_DATA_PACKET:
            printf("SCO_DATA_PACKET\n");
            if (READ_SCO_CONNECTION_HANDLE(event) != sco_handle) break;
            sco_receive(event, event_size);
            break;

        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(event)){
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(event) != HCI_STATE_WORKING) break;
                    dump_supported_codecs();
                    break;

                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(event, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_SCO_CAN_SEND_NOW:
                    printf("SCO_CAN_SEND_NOW_DATA_PACKET\n");
                    // sco_send(sco_handle);
                    break;
                    
                case HCI_EVENT_HFP_META:
                    switch (hci_event_hfp_meta_get_subevent_code(event)) {
                        case HFP_SUBEVENT_SERVICE_LEVEL_CONNECTION_ESTABLISHED:
                            status = hfp_subevent_service_level_connection_established_get_status(event);
                            if (status != ERROR_CODE_SUCCESS){
                                printf("Connection failed, status 0x%02x\n", status);
                                break;
                            }
                            acl_handle = hfp_subevent_service_level_connection_established_get_acl_handle(event);
                            hfp_subevent_service_level_connection_established_get_bd_addr(event, device_addr);
                            printf("Service level connection established %s.\n\n", bd_addr_to_str(device_addr));
                            DEBUG("ACL handle %d\n", acl_handle);
                            break;
                        case HFP_SUBEVENT_SERVICE_LEVEL_CONNECTION_RELEASED:
                            acl_handle = HCI_CON_HANDLE_INVALID;
                            printf("Service level connection released.\n\n");
                            break;
                        case HFP_SUBEVENT_AUDIO_CONNECTION_ESTABLISHED:
                            status = hfp_subevent_audio_connection_established_get_status(event);
                            if (status != ERROR_CODE_SUCCESS){
                                printf("Audio connection establishment failed with status 0x%02x\n", status);
                                break;
                            } 
                            sco_handle = hfp_subevent_audio_connection_established_get_sco_handle(event);
                            printf("Audio connection established with SCO handle 0x%04x.\n", sco_handle);
                            uint8_t negotiated_codec = hfp_subevent_audio_connection_established_get_negotiated_codec(event);
                            if (negotiated_codec != 0x02) {
                                printf("Unsupported codec 0x%02x.\n", negotiated_codec);
                                break;
                            }
                            printf("Using mSBC codec.\n");
                            sco_init_codec();
                            hci_request_sco_can_send_now_event();
                            break;

                        case HFP_SUBEVENT_CALL_ANSWERED:
                            printf("Call answered\n");
                            break;

                        case HFP_SUBEVENT_CALL_TERMINATED:
                            printf("Call terminated\n");
                            break;

                        case HFP_SUBEVENT_AUDIO_CONNECTION_RELEASED:
                            // sco_handle = HCI_CON_HANDLE_INVALID;
                            printf("Audio connection released\n");
                            // sco_demo_close();
                            break;
                        case  HFP_SUBEVENT_COMPLETE:
                            status = hfp_subevent_complete_get_status(event);
                            if (status == ERROR_CODE_SUCCESS){
                                printf("Cmd succeeded\n");
                            } else {
                                printf("Cmd failed with status 0x%02x\n", status);
                            }
                            break;

                        case HFP_SUBEVENT_AG_INDICATOR_MAPPING:
                            printf("AG Indicator Mapping | INDEX %d: range [%d, %d], name '%s'\n", 
                                hfp_subevent_ag_indicator_mapping_get_indicator_index(event), 
                                hfp_subevent_ag_indicator_mapping_get_indicator_min_range(event),
                                hfp_subevent_ag_indicator_mapping_get_indicator_max_range(event),
                                (const char*) hfp_subevent_ag_indicator_mapping_get_indicator_name(event));
                            break;

                        case HFP_SUBEVENT_AG_INDICATOR_STATUS_CHANGED:
                            printf("AG Indicator Status  | INDEX %d: status 0x%02x, '%s'\n", 
                                hfp_subevent_ag_indicator_status_changed_get_indicator_index(event), 
                                hfp_subevent_ag_indicator_status_changed_get_indicator_status(event),
                                (const char*) hfp_subevent_ag_indicator_status_changed_get_indicator_name(event));
                            break;
                        case HFP_SUBEVENT_NETWORK_OPERATOR_CHANGED:
                            printf("NETWORK_OPERATOR_CHANGED, operator mode %d, format %d, name %s\n", 
                                hfp_subevent_network_operator_changed_get_network_operator_mode(event), 
                                hfp_subevent_network_operator_changed_get_network_operator_format(event), 
                                (char *) hfp_subevent_network_operator_changed_get_network_operator_name(event));          
                            break;
                        case HFP_SUBEVENT_EXTENDED_AUDIO_GATEWAY_ERROR:
                            printf("EXTENDED_AUDIO_GATEWAY_ERROR_REPORT, status 0x%02x\n",
                                hfp_subevent_extended_audio_gateway_error_get_error(event));
                            break;
                        case HFP_SUBEVENT_START_RINGING:
                            printf("** START Ringing **\n");
                            break;
                        case HFP_SUBEVENT_RING:
                            printf("** Ring **\n");
                            break;
                        case HFP_SUBEVENT_STOP_RINGING:
                            printf("** STOP Ringing **\n");
                            break;
                        case HFP_SUBEVENT_NUMBER_FOR_VOICE_TAG:
                            printf("Phone number for voice tag: %s\n", 
                                (const char *) hfp_subevent_number_for_voice_tag_get_number(event));
                            break;
                        case HFP_SUBEVENT_SPEAKER_VOLUME:
                            printf("Speaker volume: gain %u\n",
                                hfp_subevent_speaker_volume_get_gain(event));
                            break;
                        case HFP_SUBEVENT_MICROPHONE_VOLUME:
                            printf("Microphone volume: gain %u\n",
                            hfp_subevent_microphone_volume_get_gain(event));
                            break;
                        case HFP_SUBEVENT_CALLING_LINE_IDENTIFICATION_NOTIFICATION:
                            printf("Caller ID, number '%s', alpha '%s'\n", (const char *) hfp_subevent_calling_line_identification_notification_get_number(event),
                                   (const char *) hfp_subevent_calling_line_identification_notification_get_alpha(event));
                            break;
                        case HFP_SUBEVENT_ENHANCED_CALL_STATUS:
                            printf("Enhanced call status:\n");
                            printf("  - call index: %d \n", hfp_subevent_enhanced_call_status_get_clcc_idx(event));
                            printf("  - direction : %s \n", hfp_enhanced_call_dir2str(hfp_subevent_enhanced_call_status_get_clcc_dir(event)));
                            printf("  - status    : %s \n", hfp_enhanced_call_status2str(hfp_subevent_enhanced_call_status_get_clcc_status(event)));
                            printf("  - mode      : %s \n", hfp_enhanced_call_mode2str(hfp_subevent_enhanced_call_status_get_clcc_mode(event)));
                            printf("  - multipart : %s \n", hfp_enhanced_call_mpty2str(hfp_subevent_enhanced_call_status_get_clcc_mpty(event)));
                            printf("  - type      : %d \n", hfp_subevent_enhanced_call_status_get_bnip_type(event));
                            printf("  - number    : %s \n", hfp_subevent_enhanced_call_status_get_bnip_number(event));
                            break;
                        
                        case HFP_SUBEVENT_VOICE_RECOGNITION_ACTIVATED:
                            status = hfp_subevent_voice_recognition_activated_get_status(event);
                            if (status != ERROR_CODE_SUCCESS){
                                printf("Voice Recognition Activate command failed, status 0x%02x\n", status);
                                break;
                            }
                            
                            switch (hfp_subevent_voice_recognition_activated_get_enhanced(event)){
                                case 0: 
                                    printf("\nVoice recognition ACTIVATED\n\n");
                                    break;
                                default:
                                    printf("\nEnhanced voice recognition ACTIVATED.\n");
                                    printf("Start new audio enhanced voice recognition session %s\n\n", bd_addr_to_str(device_addr));
                                    status = hfp_hf_enhanced_voice_recognition_report_ready_for_audio(acl_handle);
                                    break;
                            }
                            break;
            
                        case HFP_SUBEVENT_VOICE_RECOGNITION_DEACTIVATED:
                            status = hfp_subevent_voice_recognition_deactivated_get_status(event);
                            if (status != ERROR_CODE_SUCCESS){
                                printf("Voice Recognition Deactivate command failed, status 0x%02x\n", status);
                                break;
                            }
                            printf("\nVoice Recognition DEACTIVATED\n\n");
                            break;

                        case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_HF_READY_FOR_AUDIO:
                            status = hfp_subevent_enhanced_voice_recognition_hf_ready_for_audio_get_status(event);
                            report_status(status, "Enhanced Voice recognition: READY FOR AUDIO");
                            break;

                        case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_AG_READY_TO_ACCEPT_AUDIO_INPUT:
                            printf("\nEnhanced Voice recognition AG status: AG READY TO ACCEPT AUDIO INPUT\n\n");                            
                            break;
                        case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_AG_IS_STARTING_SOUND:
                            printf("\nEnhanced Voice recognition AG status: AG IS STARTING SOUND\n\n");                            
                            break;
                        case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_AG_IS_PROCESSING_AUDIO_INPUT:
                            printf("\nEnhanced Voice recognition AG status: AG IS PROCESSING AUDIO INPUT\n\n");                            
                            break;
                        
                        case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_AG_MESSAGE:
                            printf("\nEnhanced Voice recognition AG message: \'%s\'\n", hfp_subevent_enhanced_voice_recognition_ag_message_get_text(event));                            
                            break;

                        case HFP_SUBEVENT_ECHO_CANCELING_AND_NOISE_REDUCTION_DEACTIVATE:
                            status = hfp_subevent_echo_canceling_and_noise_reduction_deactivate_get_status(event);
                            report_status(status, "Echo Canceling and Noise Reduction Deactivate");
                            break;
                        default:
                            DEBUG("Unknown hci event_packet type: %d\n", hci_event_packet_get_type((event)));
                            break;
                    }
                    break;

                default:
                    break;
            }
            break;

        default:
            DEBUG("Unknown packet_type %d\n", packet_type);
            break;
    }

}


static void show_usage(void){
    bd_addr_t iut_address;
    gap_local_bd_addr(iut_address);

    printf("\n--- Bluetooth HFP Hands-Free (HF) unit Test Console %s ---\n", bd_addr_to_str(iut_address));
    printf("\n");
    printf("a - establish SLC to %s     | ", bd_addr_to_str(device_addr));
    printf("A - release SLC connection to device\n");
    printf("b - establish Audio connection             | B - release Audio connection\n");
    printf("d - query network operator                 | D - Enable HFP AG registration status update via bitmap(IIA)\n");
    printf("f - answer incoming call                   | F - Hangup call\n");
    printf("g - query network operator name            | G - reject incoming call\n");
    printf("i - dial 1234567                           | I - dial 7654321\n");
    printf("j - dial #1                                | J - dial #99\n");
    printf("o - Set speaker volume to 0  (minimum)     | O - Set speaker volume to 9  (default)\n");
    printf("p - Set speaker volume to 12 (higher)      | P - Set speaker volume to 15 (maximum)\n");
    printf("q - Set microphone gain to 0  (minimum)    | Q - Set microphone gain to 9  (default)\n");
    printf("s - Set microphone gain to 12 (higher)     | S - Set microphone gain to 15 (maximum)\n");
    printf("t - terminate connection\n");
    printf("u - send 'user busy' (TWC 0)\n");
    printf("U - end active call and accept other call' (TWC 1)\n");
    printf("v - Swap active call call (TWC 2)          | V - Join held call (TWC 3)\n");
    printf("w - Connect calls (TWC 4)                  | W - redial\n");
    printf("m - deactivate echo canceling and noise reduction\n");
    printf("c/C - disable/enable registration status update for all AG indicators\n");
    printf("e/E - disable/enable reporting of the extended AG error result code\n");
    printf("k/K - deactivate/activate call waiting notification\n");
    printf("l/L - deactivate/activate calling line notification\n");
    printf("n/N - deactivate/activate voice recognition\n");
    
    printf("0123456789#*-+ - send DTMF dial tones\n");
    printf("x - request phone number for voice tag     | X - current call status (ECS)\n");
    printf("y - release call with index 2 (ECC)        | Y - private consultation with call 2(ECC)\n");
    printf("[ - Query Response and Hold status (RHH ?) | ] - Place call in a response and held state(RHH 0)\n");
    printf("{ - Accept held call(RHH 1)                | } - Reject held call(RHH 2)\n");
    printf("? - Query Subscriber Number (NUM)\n");
    printf("! - Update HF indicator with assigned number 1 (HFI)\n");
    printf("\n");
}

static void stdin_process(char c){
    uint8_t status = ERROR_CODE_SUCCESS;

    cmd = c;    // used in packet handler

    if (cmd >= '0' && cmd <= '9'){
        printf("DTMF Code: %c\n", cmd);
        status = hfp_hf_send_dtmf_code(acl_handle, cmd);
        return;
    }

    switch (cmd){
        case '#':
        case '-':
        case '+':
        case '*':
            log_info("USER:\'%c\'", cmd);
            printf("DTMF Code: %c\n", cmd);
            status = hfp_hf_send_dtmf_code(acl_handle, cmd);
            break;
        case 'a':
            log_info("USER:\'%c\'", cmd);
            printf("Establish Service level connection to device with Bluetooth address %s...\n", bd_addr_to_str(device_addr));
            status = hfp_hf_establish_service_level_connection(device_addr);
            break;
        case 'A':
            log_info("USER:\'%c\'", cmd);
            printf("Release Service level connection.\n");
            status = hfp_hf_release_service_level_connection(acl_handle);
            break;
        case 'b':
            log_info("USER:\'%c\'", cmd);
            printf("Establish Audio connection to device with Bluetooth address %s...\n", bd_addr_to_str(device_addr));
            status = hfp_hf_establish_audio_connection(acl_handle);
            break;
        case 'B':
            log_info("USER:\'%c\'", cmd);
            printf("Release Audio service level connection.\n");
            status = hfp_hf_release_audio_connection(acl_handle);
            break;
        case 'C':
            log_info("USER:\'%c\'", cmd);
            printf("Enable registration status update for all AG indicators.\n");
            status = hfp_hf_enable_status_update_for_all_ag_indicators(acl_handle);
            break;
        case 'c':
            log_info("USER:\'%c\'", cmd);
            printf("Disable registration status update for all AG indicators.\n");
            status = hfp_hf_disable_status_update_for_all_ag_indicators(acl_handle);
            break;
        case 'D':
            log_info("USER:\'%c\'", cmd);
            printf("Set HFP AG registration status update for individual indicators (0111111).\n");
            status = hfp_hf_set_status_update_for_individual_ag_indicators(acl_handle, 63);
            break;
        case 'd':
            log_info("USER:\'%c\'", cmd);
            printf("Query network operator.\n");
            status = hfp_hf_query_operator_selection(acl_handle);
            break;
        case 'E':
            log_info("USER:\'%c\'", cmd);
            printf("Enable reporting of the extended AG error result code.\n");
            status = hfp_hf_enable_report_extended_audio_gateway_error_result_code(acl_handle);
            break;
        case 'e':
            log_info("USER:\'%c\'", cmd);
            printf("Disable reporting of the extended AG error result code.\n");
            status = hfp_hf_disable_report_extended_audio_gateway_error_result_code(acl_handle);
            break;
        case 'f':
            log_info("USER:\'%c\'", cmd);
            printf("Answer incoming call.\n");
            status = hfp_hf_answer_incoming_call(acl_handle);
            break;
        case 'F':
            log_info("USER:\'%c\'", cmd);
            printf("Hangup call.\n");
            status = hfp_hf_terminate_call(acl_handle);
            break;
        case 'G':
            log_info("USER:\'%c\'", cmd);
            printf("Reject incoming call.\n");
            status = hfp_hf_reject_incoming_call(acl_handle);
            break;
        case 'g':
            log_info("USER:\'%c\'", cmd);
            printf("Query operator.\n");
            status = hfp_hf_query_operator_selection(acl_handle);
            break;
        case 't':
            log_info("USER:\'%c\'", cmd);
            printf("Terminate HCI connection.\n");
            gap_disconnect(acl_handle);
            break;
        case 'i':
            log_info("USER:\'%c\'", cmd);
            printf("Dial 333\n");
            status = hfp_hf_dial_number(acl_handle, "333");
            break;
        case 'I':
            log_info("USER:\'%c\'", cmd);
            printf("Dial 7654321\n");
            status = hfp_hf_dial_number(acl_handle, "7654321");
            break;
        case 'j':
            log_info("USER:\'%c\'", cmd);
            printf("Dial #1\n");
            status = hfp_hf_dial_memory(acl_handle,1);
            break;
        case 'J':
            log_info("USER:\'%c\'", cmd);
            printf("Dial #99\n");
            status = hfp_hf_dial_memory(acl_handle,99);
            break;
        case 'k':
            log_info("USER:\'%c\'", cmd);
            printf("Deactivate call waiting notification\n");
            status = hfp_hf_deactivate_call_waiting_notification(acl_handle);
            break;
        case 'K':
            log_info("USER:\'%c\'", cmd);
            printf("Activate call waiting notification\n");
            status = hfp_hf_activate_call_waiting_notification(acl_handle);
            break;
        case 'l':
            log_info("USER:\'%c\'", cmd);
            printf("Deactivate calling line notification\n");
            status = hfp_hf_deactivate_calling_line_notification(acl_handle);
            break;
        case 'L':
            log_info("USER:\'%c\'", cmd);
            printf("Activate calling line notification\n");
            status = hfp_hf_activate_calling_line_notification(acl_handle);
            break;
        case 'm':
            log_info("USER:\'%c\'", cmd);
            printf("Deactivate echo canceling and noise reduction\n");
            status = hfp_hf_deactivate_echo_canceling_and_noise_reduction(acl_handle);
            break;
        case 'n':
            log_info("USER:\'%c\'", cmd);
            printf("Deactivate voice recognition\n");
            status = hfp_hf_deactivate_voice_recognition(acl_handle);
            break;
        case 'N':
            log_info("USER:\'%c\'", cmd);
            printf("Activate voice recognition %s\n", bd_addr_to_str(device_addr));
            status = hfp_hf_activate_voice_recognition(acl_handle);
            break;
        case 'o':
            log_info("USER:\'%c\'", cmd);
            printf("Set speaker gain to 0 (minimum)\n");
            status = hfp_hf_set_speaker_gain(acl_handle, 0);
            break;
        case 'O':
            log_info("USER:\'%c\'", cmd);
            printf("Set speaker gain to 9 (default)\n");
            status = hfp_hf_set_speaker_gain(acl_handle, 9);
            break;
        case 'p':
            log_info("USER:\'%c\'", cmd);
            printf("Set speaker gain to 12 (higher)\n");
            status = hfp_hf_set_speaker_gain(acl_handle, 12);
            break;
        case 'P':
            log_info("USER:\'%c\'", cmd);
            printf("Set speaker gain to 15 (maximum)\n");
            status = hfp_hf_set_speaker_gain(acl_handle, 15);
            break;
        case 'q':
            log_info("USER:\'%c\'", cmd);
            printf("Set microphone gain to 0\n");
            status = hfp_hf_set_microphone_gain(acl_handle, 0);
            break;
        case 'Q':
            log_info("USER:\'%c\'", cmd);
            printf("Set microphone gain to 9\n");
            status = hfp_hf_set_microphone_gain(acl_handle, 9);
            break;
        case 's':
            log_info("USER:\'%c\'", cmd);
            printf("Set microphone gain to 12\n");
            status = hfp_hf_set_microphone_gain(acl_handle, 12);
            break;
        case 'S':
            log_info("USER:\'%c\'", cmd);
            printf("Set microphone gain to 15\n");
            status = hfp_hf_set_microphone_gain(acl_handle, 15);
            break;
        case 'u':
            log_info("USER:\'%c\'", cmd);
            printf("Send 'user busy' (Three-Way Call 0)\n");
            status = hfp_hf_user_busy(acl_handle);
            break;
        case 'U':
            log_info("USER:\'%c\'", cmd);
            printf("End active call and accept waiting/held call (Three-Way Call 1)\n");
            status = hfp_hf_end_active_and_accept_other(acl_handle);
            break;
        case 'v':
            log_info("USER:\'%c\'", cmd);
            printf("Swap active call and hold/waiting call (Three-Way Call 2)\n");
            status = hfp_hf_swap_calls(acl_handle);
            break;
        case 'V':
            log_info("USER:\'%c\'", cmd);
            printf("Join hold call (Three-Way Call 3)\n");
            status = hfp_hf_join_held_call(acl_handle);
            break;
        case 'w':
            log_info("USER:\'%c\'", cmd);
            printf("Connect calls (Three-Way Call 4)\n");
            status = hfp_hf_connect_calls(acl_handle);
            break;
        case 'W':
            log_info("USER:\'%c\'", cmd);
            printf("Redial\n");
            status = hfp_hf_redial_last_number(acl_handle);
            break;
        case 'x':
            log_info("USER:\'%c\'", cmd);
            printf("Request phone number for voice tag\n");
            status = hfp_hf_request_phone_number_for_voice_tag(acl_handle);
            break;
        case 'X':
            log_info("USER:\'%c\'", cmd);
            printf("Query current call status\n");
            status = hfp_hf_query_current_call_status(acl_handle);
            break;
        case 'y':
            log_info("USER:\'%c\'", cmd);
            printf("Release call with index 2\n");
            status = hfp_hf_release_call_with_index(acl_handle, 2);
            break;
        case 'Y':
            log_info("USER:\'%c\'", cmd);
            printf("Private consultation with call 2\n");
            status = hfp_hf_private_consultation_with_call(acl_handle, 2);
            break;
        case '[':
            log_info("USER:\'%c\'", cmd);
            printf("Query Response and Hold status (RHH ?)\n");
            status = hfp_hf_rrh_query_status(acl_handle);
            break;
        case ']':
            log_info("USER:\'%c\'", cmd);
            printf("Place call in a response and held state (RHH 0)\n");
            status = hfp_hf_rrh_hold_call(acl_handle);
           break;
        case '{':
            log_info("USER:\'%c\'", cmd);
            printf("Accept held call (RHH 1)\n");
            status = hfp_hf_rrh_accept_held_call(acl_handle);
            break;
        case '}':
            log_info("USER:\'%c\'", cmd);
            printf("Reject held call (RHH 2)\n");
            status = hfp_hf_rrh_reject_held_call(acl_handle);
            break;
        case '?':
            log_info("USER:\'%c\'", cmd);
            printf("Query Subscriber Number\n");
            status = hfp_hf_query_subscriber_number(acl_handle);
            break;
        case '!':
            log_info("USER:\'%c\'", cmd);
            printf("Update HF indicator with assigned number 1 (HFI)\n");
            status = hfp_hf_set_hf_indicator(acl_handle, 1, 1);
            break;
        default:
            show_usage();
            break;
    }

    if (status != ERROR_CODE_SUCCESS){
        printf("Could not perform command, status 0x%02x\n", status);
    }
}

void bt_init() {
    // sco_demo_init();
    // gap_secure_connections_enable(false);
                                        
    gap_discoverable_control(1);
    gap_connectable_control(1);
    gap_set_class_of_device(0x200408);   
    gap_set_local_name("746 Headset 00:00:00:00:00:00");

    // init L2CAP
    l2cap_init();

    // uint16_t hf_supported_features          =
    //     (1<<HFP_HFSF_ESCO_S4)               |
    //     (1<<HFP_HFSF_CODEC_NEGOTIATION);
    
    
    uint16_t hf_supported_features          =
        (1<<HFP_HFSF_ESCO_S4)               |
        (1<<HFP_HFSF_CLI_PRESENTATION_CAPABILITY) |
        (1<<HFP_HFSF_HF_INDICATORS)         |
        (1<<HFP_HFSF_CODEC_NEGOTIATION)     |
        (1<<HFP_HFSF_ENHANCED_CALL_STATUS)  |
        (1<<HFP_HFSF_VOICE_RECOGNITION_FUNCTION)  |
        (1<<HFP_HFSF_ENHANCED_VOICE_RECOGNITION_STATUS) |
        (1<<HFP_HFSF_VOICE_RECOGNITION_TEXT) |
        (1<<HFP_HFSF_EC_NR_FUNCTION) |
        (1<<HFP_HFSF_REMOTE_VOLUME_CONTROL);

    int wide_band_speech = 1;

    rfcomm_init();
    hfp_hf_init(rfcomm_channel_nr);
    hfp_hf_init_supported_features(hf_supported_features);
    hfp_hf_init_hf_indicators(sizeof(indicators)/sizeof(uint16_t), indicators);
    hfp_hf_init_codecs(sizeof(codecs), codecs);
    
    sdp_init();    
    memset(hfp_service_buffer, 0, sizeof(hfp_service_buffer));
    hfp_hf_create_sdp_record(hfp_service_buffer, sdp_create_service_record_handle(),
                             rfcomm_channel_nr, hfp_hf_service_name, hf_supported_features, wide_band_speech);
    printf("SDP service record size: %u\n", de_get_len(hfp_service_buffer));
    sdp_register_service(hfp_service_buffer);

    // register for HCI events and SCO packets
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    hci_register_sco_packet_handler(&packet_handler);

    // Service Class: Audio, Major Device Class: Audio, Minor: Hands-Free device
    gap_set_class_of_device(0x200408);

    // allow for role switch in general and sniff mode
    gap_set_default_link_policy_settings( LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE );

    // allow for role switch on outgoing connections - this allows HFP AG, e.g. smartphone, to become master when we re-connect to it
    gap_set_allow_role_switch(true);

    // register for HFP events
    hfp_hf_register_packet_handler(packet_handler);
    
#ifdef BT_DEBUG_MODE
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
#endif

    hci_set_sco_voice_setting(0x60);    // linear, unsigned, 16-bit, CVSD

    // parse human readable Bluetooth address
    sscanf_bd_addr("B4:1B:B0:5F:DA:30", device_addr);
    // sscanf_bd_addr("38:80:DF:9B:42:6C", device_addr);
    btstack_stdin_setup(stdin_process);

    // turn on!
    hci_power_control(HCI_POWER_ON);
}