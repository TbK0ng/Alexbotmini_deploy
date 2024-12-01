from fi_fsa import fi_fsa_v2
import time
from utils import clear_screen
server_ip_list = []

def main():
    server_ip_list = fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
    clear_screen()
    if server_ip_list:
        for i in range(len(server_ip_list)):
            position, velocity, current = fi_fsa_v2.fast_get_pvc(server_ip_list[i])
            print(
                "Position = %f, Velocity = %f, Current = %.4f"
                % (position, velocity, current)
            )
            

if __name__ == "__main__":
    main()
