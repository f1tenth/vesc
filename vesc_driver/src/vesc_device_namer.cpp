#include <vesc_driver/vesc_device_uuid_lookup.hpp>

#include <stdlib.h> 
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv)
{
   std::string devicePort=  (argc>0? argv[1]:"");
   std::string VESC_UUID_ENV="VESC_UUID_ENV=";

   vesc_driver::VescDeviceLookup lookup("/dev/"+devicePort);

   for(int i=0;i<50;i++){
      usleep(20);
      if (lookup.isReady()) break;

    }
    
    if(lookup.isReady())
   {
      VESC_UUID_ENV += lookup.deviceUUID(); 
      //std::cout << VESC_UUID_ENV <<std::endl;
      std::cout<<lookup.deviceUUID();
      setenv("VESC_UUID_ENV",  lookup.deviceUUID(),0 );
      return 0;
   } else return -1;
}