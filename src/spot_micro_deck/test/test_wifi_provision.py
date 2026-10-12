import yaml

from spot_micro_deck.wifi_provision import (netplan_with_network, netplan_without_network,
                                             valid_credentials)


def test_adds_a_network_to_an_empty_file():
    doc = yaml.safe_load(netplan_with_network('', 'wlan0', 'Deck Hotspot', 'secret123'))
    wifi = doc['network']['wifis']['wlan0']
    assert doc['network']['version'] == 2
    assert wifi['dhcp4'] is True
    assert wifi['access-points'] == {'Deck Hotspot': {'password': 'secret123'}}


def test_keeps_other_networks_and_updates_the_password():
    text = netplan_with_network('', 'wlan0', 'home', 'password1')
    text = netplan_with_network(text, 'wlan0', 'phone', 'password2')
    text = netplan_with_network(text, 'wlan0', 'home', 'newpassword')
    aps = yaml.safe_load(text)['network']['wifis']['wlan0']['access-points']
    assert aps == {'home': {'password': 'newpassword'}, 'phone': {'password': 'password2'}}


def test_open_network_has_no_password():
    aps = yaml.safe_load(netplan_with_network('', 'wlan0', 'cafe', ''))['network']['wifis']['wlan0']['access-points']
    assert aps == {'cafe': {}}


def test_remove_network():
    text = netplan_with_network(netplan_with_network('', 'wlan0', 'a', 'password1'), 'wlan0', 'b', 'password2')
    aps = yaml.safe_load(netplan_without_network(text, 'wlan0', 'b'))['network']['wifis']['wlan0']['access-points']
    assert list(aps) == ['a']
    assert netplan_without_network('', 'wlan0', 'x') == ''


def test_credential_checks():
    assert valid_credentials('home', 'password1') == ''
    assert valid_credentials('home', '') == ''              # open network
    assert valid_credentials('home', 'a' * 64) == ''        # raw hex PSK
    assert valid_credentials('home', 'short')
    assert valid_credentials('', 'password1')
    assert valid_credentials('x' * 33, 'password1')
    assert valid_credentials(None, 'password1')
