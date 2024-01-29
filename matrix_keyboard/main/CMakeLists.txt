set(SOURCES
"main.cpp"
"./libs/UniversalLCD/universallcd.cpp"
"./libs/brainfvck/brainfck.cpp"
)

set(INC_DIR
"."
"libs"
"libs/UniversalLCD"
"libs/brainfvck")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ${INC_DIR})