import sys
sys.path.append('../')
from pycore.tikzeng import *

# defined your arch
arch = [
    to_head( '..' ),
    to_cor(),
    to_begin(),
    to_Conv("conv_3d_1", 4, 4, offset="(0,0,0)", to="(0,0,0)", height=64, depth=64, width=32 ),
    to_Pool("pool_3d_1", offset="(0,0,0)", to="(conv1-east)"),
    to_connection( "pool_3d_1", "conv_3d_2"),

    to_Conv("conv_3d_2", 128, 64, offset="(1,0,0)", to="(pool_3d_1-east)", height=32, depth=32, width=32 ),
    to_Pool("pool_3d_2", offset="(0,0,0)", to="(conv_3d_2-east)"),
    to_connection( "pool_3d_2", "conv2"),

    to_Conv("conv_3d_3", 128, 64, offset="(1,0,0)", to="(pool_3d_2-east)", height=32, depth=32, width=32),
    to_Pool("pool_3d_3", offset="(0,0,0)", to="(conv_3d_3-east)"),
    to_Pool("pool2", offset="(0,0,0)", to="(conv2-east)", height=28, depth=28, width=1),
    to_SoftMax("soft1", 10 ,"(3,0,0)", "(pool1-east)", caption="SOFT"  ),
    to_connection("pool2", "soft1"),
    to_end()
    ]

def main():
    namefile = str(sys.argv[0]).split('.')[0]
    to_generate(arch, namefile + '.tex' )

if __name__ == '__main__':
    main()