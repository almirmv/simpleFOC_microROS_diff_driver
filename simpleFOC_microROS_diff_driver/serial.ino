void initSerial(){
  Serial.begin(115200);
  delay(1000);
}

void printInfoSerial(){
  Serial.print("[INFO] ");
}

//============
void sendParseOkSerial(){
  Serial.println("OK"); 
  }
void sendParseErrorSerial(){
  Serial.println("PE"); 
  }  
