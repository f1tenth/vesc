#include "vesc_driver/vesc_device_uuid_lookup.hpp"

#include <cassert>
#include <cmath>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <boost/bind.hpp>

namespace vesc_driver
{


VescDeviceLookup::VescDeviceLookup(std::string name)
:
  vesc_(0,
        std::string(),
        boost::bind(&VescDeviceLookup::vescPacketCallback, this, _1),
        boost::bind(&VescDeviceLookup::vescErrorCallback, this, _1)
       ),
  ready_(false),
  device_(name)
       {
           //std::cout<<"device port " << device_ << std::endl;

           try {
                vesc_.connect(device_);
                vesc_.requestFWVersion(0);
              }
              catch (SerialException e) {
                std::cerr << "VESC error on port "<<device_ << std::endl << e.what() <<std::endl;
                return;
              }
       }
   
void VescDeviceLookup::vescPacketCallback(const std::shared_ptr<VescPacket const>& packet){
       if (packet->name() == "FWVersion") {
            std::shared_ptr<VescPacketFWVersion const> fw_version =
              std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
            // todo: might need lock here
            const uint8_t *uuid= fw_version->uuid();
            
            hwname_ = fw_version->hwname();
            version_ = fw_version->fwMajor() + "." + fw_version->fwMinor();
            char uuid_data[100] ="";
            //for(int i=0 ;i<12 ;i++)
            //std::cout << std::hex << std::setfill ('0') << std::setw(2) << (int)uuid[i] << " ";

            //std::cout << std::endl;
             
            sprintf(uuid_data, 
                        "%02x%02x%02x-%02x%02x%02x-%02x%02x%02x-%02x%02x%02x", 
                            uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], 
                            
                            uuid[6], uuid[7], uuid[8], uuid[9], uuid[10], uuid[11]  
                        );
             
             uuid_ += uuid_data;
             ready_=true;
          }
   }
   
void VescDeviceLookup::vescErrorCallback(const std::string& error){
       error_ = error;
       ready_=false;
   }

bool VescDeviceLookup::isReady(){
    return ready_;
}

const char* VescDeviceLookup::deviceUUID() const{
      return uuid_.c_str();
   }


    const char* VescDeviceLookup::version() const{

        return version_.c_str();
    }
    const char* VescDeviceLookup::hwname() const{
        return hwname_.c_str();
    } 
} 