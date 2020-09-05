//#include <ros.h>
//#include <std_msgs/String.h>
//
//ros::NodeHandle listener;
//std_msgs::String str_msg;
//ros::Publisher answer("answer", &str_msg);
//char hello[13] = "hello world!";
//
//void messageROS( const std_msgs::String& toggle_msg);
//ros::Subscriber<std_msgs::String> sub("talker" , &messageROS);
//
//void setupROS()
//{
//  pinMode(13, OUTPUT);  
//  listener.initNode();
//  listener.subscribe(sub);
//
//  listener.initNode();
//  listener.advertise(answer);  
//}
//
//void messageROS( const std_msgs::String& toggle_msg)
//{
//  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
//  digitalWrite(13, HIGH);   // blink the led
//  
//  if(toggle_msg.data[0] == '9')
//  {
//    str_msg.data = hello;
//    answer.publish( &str_msg );
//    listener.spinOnce();
//    digitalWrite(13, LOW);   // blink the led
//    delay(5000);
//  }
//}

//void loop()
//{  
//  listener.spinOnce();
//  delay(1);
//}
