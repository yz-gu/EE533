import os

RV64I_REQ_RW_ADDR_IN_REG  = 0x2000200 # [31:28]: 8:enable, 6:write request, 4:read request, 1:reset, 0:disable; [27:12]:16'h0000; [11:0]:000-1ff:inst mem, 200-2ff:data mem
RV64I_DIN_HIGH_REG        = 0x2000204
RV64I_DIN_LOW_REG         = 0x2000208
RV64I_REQ_RW_ADDR_OUT_REG = 0x200020c
RV64I_DOUT_HIGH_REG       = 0x2000210
RV64I_DOUT_LOW_REG        = 0x2000214

foutlog = open('./instr_load.log', 'w')

def instr_load():
    fin = open('./machinecode.txt','r')
    print('loading instructions', file=foutlog)
    
    for num, instr in enumerate(fin.readlines()):
        os.system('regwrite ' + INSTR_ADDR + ' ' + hex(num))
        os.system('regwrite ' + INSTR + ' ' + instr.strip())

        print('regwrite ' + INSTR_ADDR + ' ' + hex(num), file=foutlog)
        print('regwrite ' + INSTR + ' ' + instr.strip(), file=foutlog)
        print(' ', file = foutlog)
    
def pipeline_disable():
    print('disabling pipeline', file = foutlog)
    pipeline_disable = '0x0000000'
    os.system('regwrite ' + INSTR_ADDR + pipeline_disable)
    print('regwrite ' + INSTR_ADDR +' '+ pipeline_disable, file=foutlog)

def data_write():
    data_wr_en = 0x100
    data_high = ['0x0','0x0','0x0','0x0','0x0']
    data_low = ['0x2', '0x2000', '4000','0x0','0x0']
    
    # number should be less than 256
    # number = data_high.__len__() < data_low.__len__() ?data_high.__len__(): data_low.__len__()
    number = data_high.__len__() if (data_high.__len__() < data_low.__len__()) else data_low.__len__()

    for num in range(number):
        os.system('regwrite' + DATA_ADDR + str(data_wr_en +num))
        os.system('regwrite' + DATA_HIGH + data_high[num])
        os.system('regwrite' + DATA_LOW + data_low[num])
        # print('regwrite' + DATA_ADDR + str(data_wr_en + num))
        # print('regwrite' + DATA_HIGH + data_high[num])
        # print('regwrite' + DATA_LOW + data_low[num])

def data_write_disable():
    data_wr_disable = '0x000'
    os.system('regwrite' + DATA_ADDR + data_wr_disable)
    
if __name__ == "__main__":
    pipeline_disable()
    instr_load()
    data_write()
    data_write_disable()
