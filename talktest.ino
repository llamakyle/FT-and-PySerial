
String rd;
int x;
int y;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {




//Read new data as it is sent
  if (Serial.available() > 0) {
    // read the incoming byte:
    rd = Serial.readStringUntil(',');
    y+=rd.toInt();
    x += Serial.parseInt();

    Serial.println(x);

    }    
  

  

  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(3); 
  Serial.print("\t");
  Serial.println(4); 

delay(10);
}
