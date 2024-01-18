set(SOURCES
"main.cpp"
"./tp_includes/UniversalLCD/universallcd.cpp"
)

set(INC_DIR
"."
"tp_includes"
"tp_includes/UniversalLCD")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ${INC_DIR})
