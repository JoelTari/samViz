/*
 * Copyright 2021 AKKA Technologies (joel.tari-summerfield@akka.eu)
 *
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by
 * the European Commission - subsequent versions of the EUPL (the "Licence");
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence at:
 *
 * https://joinup.ec.europa.eu/software/page/eupl
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the Licence is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the Licence for the specific language governing permissions and
 * limitations under the Licence.
 */

#include "Network/Clients/MQTT.hpp"
#include <iostream>

// Example using eMQTT5 library 
//         https://github.com/X-Ryl669/eMQTT5

// To compile with g++, do:
//      g++ graph_publish.cpp -o graph_publish -leMQTT5 -L{PATH_TO_STATIC_LIB_OBJ} -I {eMQTT5_DIRECTORY}/lib/include
//
// PATH_TO_STATIC_LIB_OBJ could be '/usr/local/lib' '$HOME/.local/lib' '{eMQTT5_DIRECTORY}/lib/include'
//                        depending on how you compile the source code of eMQTT5
// eMQTT5_DIRECTORY is path where you cloned the eMQTT5 repository



// This class processes the incomming messages
// the msgReceive function should implement the callback
class ImplMsgReceived : public Network::Client::MessageReceived
{
  void messageReceived(const DynamicStringView &  topic,
                       const DynamicBinDataView & payload,
                       const uint16               packetIdentifier,
                       const PropertiesView &     properties)
  {
    std::cout << "Message Received, processing...\n";

    // if (topic == 'a_topic_I_subscribed_to')
    //      do_something();

    std::cout << "A message has been processed\n";

    
  }
};

// main function
int main()
{

  // The class that consumes the incoming messages
  ImplMsgReceived MsgConsumer;

  // Create an MQTT client, and bind to the message consumer
  Network::Client::MQTTv5 mqttobj("cppCode", &MsgConsumer);

  // Connect the mqtt client to the broker, broker is assumed to e at localhost
  std::cout << "Connect to broker :  " << mqttobj.connectTo("localhost", 1883)
            << "\n";

  // we can make some subscriptions 
  // mqttobj.subscribe("some_topic_name");

  const char * msg = "I'm broadcasting a msg on the 'presence' topic !";
  //json versoin
  /* const char * msg = "{\"presence\": \"I'm broadcasting a msg on the 'presence' topic !\"}"; */
  mqttobj.publish("presence", (uint8 *)msg, strlen(msg));

  // event loop: here it is blocking
  //            there are other methods to call a non-blocking function
  while(1){
    mqttobj.eventLoop();
    usleep(10000);
  }

  return 0;
}
