/*
 * 12giu17  aggiunta selezione rete via jumper
 *          led rosso usato per indicare ricerca rete (blink)
 *          rete trovata (acceso)
 *          
 */
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>

SoftwareSerial swSer(14, 12, false, 256);

//**********aggiunte
//#include <SoftwareSerial.h>
//**********fine aggiunte
//seriale di comunicazione con esp uso pin 7 e 9 
//SoftwareSerial ESPserial(7, 9); // RX | TX
//**********fine aggiunte


String webPage ;


const char* ssid1     = "";
const char* password1 = "";

const char* ssid      = "";
const char* password  = "";

//const char* ssid = "BiblioWiFi";
//const char* password = "";

//const char* ssid = "Pi_AP";
//const char* password = "Raspberry";
//const char* ssid = "TP-LINK_C508BC";
//const char* password = "wolfgang";
//IPAddress fixed_ip(192,168,42,18);
//IPAddress dns_ip(8,8,8,8);
//IPAddress gw_ip(192,168,42,1);

ESP8266WebServer server(80);

const int led = LED_BUILTIN;
HTTPClient http;

void provaaiax()
{
   webPage="<html>  <head>  <script src=\"http://code.jquery.com/jquery-latest.js\"></script>  <script type=\"text/javascript\">  $(document).ready(function() {  $(\"form#iscrizione\").submit(function(){ ";
webPage+="  var nome = $(\"#nome\").val();  var cognome = $(\"#cognome\").val();  $.ajax({  type: \"GET\",  url: \"/risp\",  data: \"nome=\" + nome + \"&cognome=\" + cognome,";
webPage+="  dataType: \"html\",  success: function(risposta) {  $(\"div#risposta\").html(risposta);  },  error: function(){  alert(\"Chiamata fallita!!!\");  }  });  return false;  }); }); </script> ";
webPage+="  </head> <body>  <form id=\"iscrizione\"> <p> Inserisci il nome:<br/> <input type=\"text\" name=\"nome\" id=\"nome\"/> </p> <p> Inserisci il cognome:<br/> <input type=\"text\" name=\"cognome\" id=\"cognome\"/>";
webPage+=" </p> <p> <input type=\"submit\" value=\"invia\"></p></form> <div id=\"risposta\"></div>  </body> </html> ";
 server.send(200, "text/html",webPage ); 
  
  }
void handleRoot() {
  digitalWrite(led, 1);
  server.send(200, "text/plain", "hello from esp8266!");
  digitalWrite(led, 0);
}
void risposta() {
  digitalWrite(led, 1);
  
  Serial.println(server.arg(0)); 
  delay(2);  

     int okcom;
     int nonFinito;
     long timeStart;
     String _risposta = "";

     timeStart = millis();
     okcom=0;
     nonFinito = 1;   
     while (nonFinito){
          if ((millis() - timeStart) > 2000){
            nonFinito = 0;
            _risposta = "timeout";
          }
          
         if(Serial.available() >0){
            char inChar= Serial.read();
            //Serial.println(inChar);
            if (inChar  == '!')     // carattere iniziale
            { _risposta = "";
              okcom=1;
            }
        
            if (inChar  != '?' && okcom==1){ 
              _risposta += inChar;
            }
            if (inChar  == '?' ){ 
              okcom=0;
              nonFinito = 0;
            }
        }
     }// end while

  server.send(200, "text/plain", "answ: " + _risposta);
  _risposta="";
  digitalWrite(led, 0);
}

void handleNotFound(){
  digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
}



void setup(void){
  pinMode(led, OUTPUT);
  pinMode(D5, INPUT_PULLUP);    // usato per selezionare rete wifi
  
  digitalWrite(led, 0);
  Serial.begin(9600);  //usb verso arduino
   swSer.begin(9600);   // seriale sw
  /*
   * Serial uses UART0, which is mapped to pins GPIO1 (TX) and GPIO3 (RX). 
   * Serial may be remapped to GPIO15 (TX) and GPIO13 (RX) by calling Serial.swap(); after Serial.begin();. 
   * Calling swap again maps UART0 back to GPIO1 and GPIO3. - 
   * See more at: http://www.esp8266.com/viewtopic.php?f=33&t=3722&sid=2d61dcd4b718226ffa0952f5f7cff53f#sthash.NUMtPCP9.dpuf
   * */
  Serial.swap();

  delay(500);
  if (digitalRead(D5)){
    WiFi.begin(ssid, password);
  }
  else{
    WiFi.begin(ssid1, password1);
  }
   
  //WiFi.config(fixed_ip, gw_ip, dns_ip);
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(led, !digitalRead(led));
  }

  digitalWrite(led, 1);

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  server.on("/aiax", provaaiax);
  server.on("/risp", risposta);
  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void){
  //http.begin("http://192.168.1.12/test.html"); //HTTP
  //http.end();
  server.handleClient();


 
  }
   
   


