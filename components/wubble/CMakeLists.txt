idf_component_register(SRCS "wubble.c" "sx1261_driver.c" 
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash bt Nimble_adv)
