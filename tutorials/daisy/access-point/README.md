# Setup an Access Point for Daisy

[Daisy]'s standard network setup uses a dedicated router
to connect with the [OptiTrack] or with other rovers.
However, you might provide the access point with your notebook too.

`daisy-pi` automatically connects to the access point `<SSID>` over its interface wlan0.
Setup a hotspot with the same SSID on your notebook to mimic the router.


## Ubuntu
(tested with 18.04, but should work similarly for <18.04 too)

With 18.04. you can turn on a default hotspot in the WiFi settings (manager),
however, the SSID is your hostname and the password is randomly generated.

You can setup network connections with:
```
$ nm-connection-editor
```

Create a new network:
* Add a connection
* Select type *WiFi*
* Give it a name `<hotspot-name>`
* Uncheck automatic connection
* Set the SSID
* Choose *Hotspot* as mode
* IP4 Settings: set your static IP according to the [network setup] with [Daisy]
* Security: WPA & WPA2 Personal and set the password

Create a route to reach `daisy`:
```
$ sudo route add -net <ne-base-ip> netmask 255.255.255.0 gw <nw-diasy-pi-ip>
```

Or add the route to `/etc/network/interfaces`:
```
# Static route
up route add -net <ne-base-ip> netmask 255.255.255.0 gw <nw-diasy-pi-ip>
```

**(In 18.04:) Do not use the WiFi settings manager to start the hotspot - it resets the SSID to the hostname!**

Start the hotspot via the command line:
```
$ nmcli connection up id <hotspot-name>
```

Note, [Daisy] has no internet connection with this simple setup.
IP forwarding similar to what `daisy-pi` is doing for `daisy` has to be installed
(see the overview to [Daisy]).

## Troubleshooting

* Check IP address: `ifconfig`
* Check WiFi is in master mode: `iwconfig`
* Check the right hotspot is up: `nmcli connection show`
* Ping `daisy-pi` and `daisy`: `ping daisy`
* Check route to `daisy` over `daisy-pi`: `route -v`
* Check the settings in `/etc/NetworkManager/system-connections/<hotspot-name>`


## References

* Overview, tutorials and demos for [Daisy]


[network setup]: ../../dagobert-network-setup.md
[Daisy]: ../README.md
[OptiTrack]: ../optitrack/README.md

---
2019-02-06 | Denise Ratasich
