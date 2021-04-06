import learning.nets.fc_2layers_1024units as fc_2layers_1024units
import learning.nets.fc_2layers_gated_1024units as fc_2layers_gated_1024units

def build_net(net_name, input_tfs, reuse=False):
    net = None

    if (net_name == fc_2layers_1024units.NAME):
        net = fc_2layers_1024units.build_net(input_tfs, reuse)
    elif (net_name == fc_2layers_gated_1024units.NAME):
        net = fc_2layers_gated_1024units.build_net(input_tfs, reuse)
    else:
        assert False, 'Unsupported net: ' + net_name
    
    return net