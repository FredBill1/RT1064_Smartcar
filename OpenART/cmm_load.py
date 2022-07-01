from pyb import Pin
import cmm
import gc
def load():
    try:
        fd = open('/sd/cmm_cfg.csv')
        print('loading /sd/cmm_cfg.csv')
    except:
        fd = open('/flash/cmm_cfg.csv')
        print('loading /flash/cmm_cfg.csv')
    dict = {}
    for sLine in fd:
        if sLine[:2] == '//':
            continue # // is comment
        # strip trailing \n or \r\n
        if sLine[-2] == '\r':
            sLine = sLine[:-2]
        else:
            sLine = sLine[:-1]
        lst = sLine.split(',')
        if len(lst) < 4:
            continue
        # cmm item format: 
        # (0)fn,(1)unit,(2)signal,(3)hint,(4)pinobj,(5)comments
        if lst[1] == '-': # no "unit" field
            comboName = lst[0] + '.' + lst[2]
        else:
            comboName = lst[0] + '.' + lst[1] + '.' + lst[2]
        try:
            # try to create a pin object with "pinobj" string
            pin = Pin(lst[4])
            # we successfully create this pin object, add it to the pin map dict
            # key is comboname, 
            #value is (hint, pinobj string, pinobj pointer, owner pointer)
            dict[comboName] = (lst[3], lst[4], pin, None)
        except:
            # no such pin object, set pinobj pointer to NULL (0)
            dict[comboName] = (lst[3], lst[4], None,   None)
        del(lst)
        del(comboName)
    fd.close()
    cmm.add(dict)
    return dict
if __name__ == '__main__':
    dict = load()
