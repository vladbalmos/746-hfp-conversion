# idf_component_register(SRCS "audio.c" "audio_test.c"
#                     INCLUDE_DIRS "." "./../../common")

idf_component_register(SRCS "dialer.c" "ringer.c" "audio.c" "bt.c" "main.c"
                    INCLUDE_DIRS "." "./../../common")
