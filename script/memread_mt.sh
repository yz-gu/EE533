echo 'Read data memory 0-5'
regwrite 0x2000200 0x40000200
regread 0x2000214
regwrite 0x2000200 0x40000201
regread 0x2000214
regwrite 0x2000200 0x40000202
regread 0x2000214
regwrite 0x2000200 0x40000203
regread 0x2000214
regwrite 0x2000200 0x40000204
regread 0x2000214
regwrite 0x2000200 0x40000205
regread 0x2000214

echo 'Read data memory 64-69'
regwrite 0x2000200 0x40000240
regread 0x2000214
regwrite 0x2000200 0x40000241
regread 0x2000214
regwrite 0x2000200 0x40000242
regread 0x2000214
regwrite 0x2000200 0x40000243
regread 0x2000214
regwrite 0x2000200 0x40000244
regread 0x2000214
regwrite 0x2000200 0x40000245
regread 0x2000214

echo 'Read data memory 128-133'
regwrite 0x2000200 0x40000280
regread 0x2000214
regwrite 0x2000200 0x40000281
regread 0x2000214
regwrite 0x2000200 0x40000282
regread 0x2000214
regwrite 0x2000200 0x40000283
regread 0x2000214
regwrite 0x2000200 0x40000284
regread 0x2000214
regwrite 0x2000200 0x40000285
regread 0x2000214

echo 'Read data memory 192-197'
regwrite 0x2000200 0x400002C0
regread 0x2000214
regwrite 0x2000200 0x400002C1
regread 0x2000214
regwrite 0x2000200 0x400002C2
regread 0x2000214
regwrite 0x2000200 0x400002C3
regread 0x2000214
regwrite 0x2000200 0x400002C4
regread 0x2000214
regwrite 0x2000200 0x400002C5
regread 0x2000214