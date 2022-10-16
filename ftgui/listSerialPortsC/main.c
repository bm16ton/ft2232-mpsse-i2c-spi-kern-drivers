#include <stdio.h>
#include <libserialport.h>

int main(void)
{
    int i;
    struct sp_port **ports;
    enum sp_return ret;

    ret = sp_list_ports(&ports);
    if (ret != SP_OK) {
        return 0;
    }

    struct sp_port *port;
    char vid_pid_iserial[256] = " ";

    for (i = 0; ports[i]; i++) {
        sp_get_port_by_name(sp_get_port_name(ports[i]), &port);

        int vid, pid;
        if (sp_get_port_usb_vid_pid(port, &vid, &pid) == SP_OK) {
            printf("%s | %04X:%04X | %s %s %s %s\n", sp_get_port_name(ports[i]), vid, pid, sp_get_port_usb_serial(port), sp_get_port_usb_product(port), sp_get_port_usb_manufacturer(port), sp_get_port_description(port));
        }
        sp_free_port(port);
    }
    sp_free_port_list(ports);
}
