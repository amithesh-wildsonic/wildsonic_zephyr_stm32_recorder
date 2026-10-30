import os

with open('app/src/main.c', 'r', encoding='utf-8') as f:
    content = f.read()

target1 = '#include "audio_format.h"\n'
repl1 = '#include "audio_format.h"\n#include "ble_comm.h"\n'

target2 = '  console_async_init(console_dev);\n  config_store_init();\n'
repl2 = (
    '  console_async_init(console_dev);\n\n'
    '  const struct device *ble_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(usart1));\n'
    '  if (ble_dev && device_is_ready(ble_dev)) {\n'
    '    ble_comm_init(ble_dev);\n'
    '  } else {\n'
    '    LOG_WRN("USART1 BLE device not available or ready");\n'
    '  }\n\n'
    '  config_store_init();\n'
)

assert target1 in content, 'target1 not found'
assert target2 in content, 'target2 not found'

content = content.replace(target1, repl1, 1)
content = content.replace(target2, repl2, 1)

with open('app/src/main.c', 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(content)

print('Successfully patched app/src/main.c')

