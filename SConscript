from building import * 

# get current dir path
cwd = GetCurrentDir()

src = []
inc = [cwd]

src += ['sensor_dallas_dht11.c']

if GetDepend(['PKG_USING_DHT11_SAMPLE']):
    src += ['dht11_sample.c']

group = DefineGroup('dht11', src, depend = ['PKG_USING_DHT11'], CPPPATH = inc)
Return('group')
