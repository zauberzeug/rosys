from rosys.actors import NetworkMonitor


def test_parsing_ip_stats():
    example_output = '''
5: wlan1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    link/ether 1c:bf:ce:3b:5c:a2 brd ff:ff:ff:ff:ff:ff
    inet 192.168.43.1/24 brd 192.168.43.255 scope global noprefixroute wlan1
       valid_lft forever preferred_lft forever
    inet6 fe80::1ebf:ceff:fe3b:5ca2/64 scope link 
       valid_lft forever preferred_lft forever
    RX: bytes  packets  errors  dropped overrun mcast   
    0          0        0       0       0       0       
    TX: bytes  packets  errors  dropped carrier collsns 
    26288      187      0       0       0       0       
6: wlan2: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    link/ether 1c:bf:ce:3b:64:65 brd ff:ff:ff:ff:ff:ff
    inet 192.168.42.2/24 brd 192.168.42.255 scope global noprefixroute wlan2
       valid_lft forever preferred_lft forever
    inet6 fe80::1ebf:ceff:fe3b:6465/64 scope link 
       valid_lft forever preferred_lft forever
    RX: bytes  packets  errors  dropped overrun mcast   
    0          0        0       0       0       0       
    TX: bytes  packets  errors  dropped carrier collsns 
    26182      187      0       0       0       0        
'''
    interfaces = NetworkMonitor.split_interfaces(example_output)
    assert len(interfaces) == 2
    assert interfaces[0].startswith('wlan1')
    assert interfaces[1].startswith('wlan2')
