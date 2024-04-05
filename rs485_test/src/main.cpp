#include <Arduino.h>
#include <SoftwareSerial.h>

#define RS485_TX 	 16 //HARDWARE SERIAL 2
#define RS485_RX     17 //HARDWARE SERIAL 2
#define RS485_EN     32
#define V_EN		 33
#define RS485_BR 	 9600//4800//


byte soilSensorResponse[9];

byte co2SensorResponse[12];

byte baudRateRequestCO2[] = {
	0x2D,    // Dirección del dispositivo (0x2D es la representación decimal de 45)
	0x06,    // Número de función (escritura de datos)
	0x00, 0x11, // Dirección del registro a escribir (0x0011 para cambio de baudrate)
	0x00, 0x02, // Índice del baudrate a establecer (Revisar datasheet para distintos valores)
	0x00, 0x00 // CRC (a rellenar)
};

byte directionRequestCO2[] = {
	0x2D,    // Dirección del dispositivo (0x2D es la representación decimal de 45)
	0x06,    // Número de función (escritura de datos)
	0x00, 0x10, // Dirección del registro a escribir (0x0010 para cambio de direccion)
	0x00, 0x02, // Direccion a establecer en hex
	0x00, 0x00 // CRC (a rellenar)
};

#define SERIAL_RS485 Serial2
SoftwareSerial swSer;


//Las funciones readMOISTURE y readCO2 fueron hechas antes de sendInstruction y readCommunitation y se pueden actualizar
void readMOISTURE()
{
	byte soilSensorRequest[] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};
	//TRY WITH 410 TRAMA
    digitalWrite(RS485_EN,HIGH);
    SERIAL_RS485.write(soilSensorRequest, sizeof(soilSensorRequest));
    SERIAL_RS485.flush();
    digitalWrite(RS485_EN,LOW);
    delay(100);

	// NUEVOOO

	unsigned long startTime = millis();
	while (SERIAL_RS485.available() < 7 && millis() - startTime < 1000)
	{
		delay(1);
	}

	if (SERIAL_RS485.available() >= 7) // If valid response received
	{
		// Read the response from the sensor
		byte index = 0;
		while (SERIAL_RS485.available() && index < 7)
		{
			soilSensorResponse[index] = SERIAL_RS485.read();
			Serial.print(soilSensorResponse[index], HEX); // Print the received byte in HEX format
			Serial.print(" ");
			index++;
		}
		Serial.println();

		// Parse and calculate the Moisture value
		int Moisture_Int = int(soilSensorResponse[3] << 8 | soilSensorResponse[4]);
		float Moisture_Percent = Moisture_Int / 10.0;

		Serial.print("Moisture: ");
		Serial.print(Moisture_Percent);
		Serial.println(" %RH\n");
	}
	else
	{
		Serial.println("Sensor timeout or incomplete frame");
	}

}

void readCO2()
{
	byte sensorRequest[] = {
        0x2D, // Dirección del dispositivo (0x2D es la representación hexadecimal de 45)
        0x03, // Número de función (Lectura de datos)
        0x00, 0x00, // Dirección de inicio del registro a leer (por ejemplo, 0x0000)
        0x00, 0x03, // Cantidad de registros a leer (2 bytes)
        0x02, 0x67 // CRC (a rellenar)
    };

    digitalWrite(RS485_EN,HIGH);
    SERIAL_RS485.write(sensorRequest, sizeof(sensorRequest));
    SERIAL_RS485.flush();
    digitalWrite(RS485_EN,LOW);
    delay(100);

	unsigned long startTime = millis();
	while (SERIAL_RS485.available() < 11 && millis() - startTime < 1000)
	{
		delay(1);
	}
	if (SERIAL_RS485.available() >= 11) // If valid response received
	{
		// Read the response from the sensor
		byte index = 0;
		while (SERIAL_RS485.available() && index < 11)
		{
			co2SensorResponse[index] = SERIAL_RS485.read();
			Serial.print(co2SensorResponse[index], HEX); // Print the received byte in HEX format
			Serial.print(" ");
			index++;
		}
		Serial.println();

		if (co2SensorResponse[0] == 0x2D && co2SensorResponse[1] == 0x03 && co2SensorResponse[2] == 0x06) {
            // Parsea los valores de CO2, Temperatura y Humedad
            int CO2 = (co2SensorResponse[3] << 8) | co2SensorResponse[4];
            int Temp = (co2SensorResponse[5] << 8) | co2SensorResponse[6];
            int Humedad = (co2SensorResponse[7] << 8) | co2SensorResponse[8];

            Serial.print("CO2: ");
            Serial.print(CO2);
            Serial.println(" ppm");

            Serial.print("Humedad: ");
            Serial.print(Humedad/100);
            Serial.println(" %");

            Serial.print("Temperatura: ");
            Serial.print(Temp/100);
            Serial.println(" C");

			Serial.println(co2SensorResponse[1]);
			Serial.println();
        } else {
            Serial.println("Error: Respuesta del sensor no válida");
        }
    } else {
        Serial.println("Error: Tiempo de espera excedido o trama incompleta");
    }

}

uint16_t calculateCRC(byte data[], uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}
/* Actualmente changeBaudRate y changedirection pueden ser reemplazadas por sendInstruction, entregando la trama correcta como entrada unicamente
void changeBaudRate() {
  // Prepara la trama de solicitud para cambiar el baudrate
  byte baudRateRequest[] = {
    0x2D,    // Dirección del dispositivo (0x2D es la representación decimal de 45)
    0x06,    // Número de función (escritura de datos)
    0x00, 0x11, // Dirección del registro a escribir (0x0011 para cambio de baudrate)
    0x00, 0x02, // Índice del baudrate a establecer (Revisar datasheet para distintos valores)
    0x00, 0x00 // CRC (a rellenar)
  };
  
  // Calcula el CRC
  uint16_t crc = calculateCRC(baudRateRequest, sizeof(baudRateRequest) - 2);
  baudRateRequest[sizeof(baudRateRequest) - 2] = crc & 0xFF; // CRC bajo
  baudRateRequest[sizeof(baudRateRequest) - 1] = crc >> 8;    // CRC alto

  Serial.println("Cambio de baudrate");
  
  // Envía la solicitud para cambiar el baudrate
  digitalWrite(RS485_EN, HIGH); // Habilita la transmisión
  SERIAL_RS485.write(baudRateRequest, sizeof(baudRateRequest)); // Envía la solicitud
  SERIAL_RS485.flush(); // Espera a que se complete la transmisión
  digitalWrite(RS485_EN, LOW); // Deshabilita la transmisión
  
  delay(100); // Espera 1 segundo para permitir que el cambio de baudrate surta efecto
}

void changedirection() {
  // Prepara la trama de solicitud para cambiar el baudrate
  byte directionRequest[] = {
    0x2D,    // Dirección del dispositivo (0x2D es la representación decimal de 45)
    0x06,    // Número de función (escritura de datos)
    0x00, 0x10, // Dirección del registro a escribir (0x0010 para cambio de direccion)
    0x00, 0x02, // Direccion a establecer
    0x00, 0x00 // CRC (a rellenar)
  };
  
  // Calcula el CRC
  uint16_t crc = calculateCRC(directionRequest, sizeof(directionRequest) - 2);
  directionRequest[sizeof(directionRequest) - 2] = crc & 0xFF; // CRC bajo
  directionRequest[sizeof(directionRequest) - 1] = crc >> 8;    // CRC alto

  Serial.println("Cambio de direccion");
  
  // Envía la solicitud para cambiar el baudrate
  digitalWrite(RS485_EN, HIGH); // Habilita la transmisión
  SERIAL_RS485.write(directionRequest, sizeof(directionRequest)); // Envía la solicitud
  SERIAL_RS485.flush(); // Espera a que se complete la transmisión
  digitalWrite(RS485_EN, LOW); // Deshabilita la transmisión
  
  delay(100); // Espera 1 segundo para permitir que el cambio de baudrate surta efecto
}
*/
template <size_t N>

void sendInstruction(int enable, Stream &RST, byte (&trama)[N]){
  digitalWrite(enable, HIGH); // Habilita la transmisión
  RST.write(trama, N); // Envía la solicitud
  RST.flush(); // Espera a que se complete la transmisión
  digitalWrite(enable, LOW); // Deshabilita la transmisión
  
  delay(100); // Espera 1 segundo para permitir que el cambio de baudrate surta efecto
}

byte* readCommunication(Stream &RST, int bits_lectura) { //ADVERTENCIA si el datasheet dice que los bits de respuesta son 12 int bits_lectura debe ser 11, es decir bits-1
    byte* response = new byte[bits_lectura];
    unsigned long startTime = millis();
    while (RST.available() < bits_lectura && millis() - startTime < 1000) {
        delay(1);
    }
    if (RST.available() >= bits_lectura) {
        byte index = 0;
        while (RST.available() && index < bits_lectura)
        {
			response[index] = RST.read();
			Serial.print(response[index], HEX); // Print the received byte in HEX format
			Serial.print(" ");
			index++;
        }
        Serial.println();
        }
    return response;
}

void setup() {

	Serial.begin(115200); //para el monitor serial
	pinMode(RS485_EN,OUTPUT); //enable de la comunicación
	pinMode(V_EN,OUTPUT);
	digitalWrite(V_EN  ,HIGH);
	digitalWrite(RS485_EN,LOW);	//define el pin de enable
	//Se realizaron las pruebas con Serial1, Serial2, y un SoftwareSerial, todas las convenciones funcionan correctamente
	SERIAL_RS485.begin(RS485_BR); //establece el baudrate de la comunicación
	//swSer.begin(RS485_BR, SWSERIAL_8N1, 27, 26);
	//Serial1.begin(RS485_BR, SERIAL_8N1, 27, 26);
}

void loop() {
	/*
	Serial.println("Medir Moisture");
	byte soilSensorRequest[] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};
	Serial.println("envia solicitud");
	sendInstruction(RS485_EN, Serial1, soilSensorRequest);
	Serial.println("Enviada...Leer");
	byte* response = readCommunication(Serial1, 7); // Obtener la respuesta

	delay(5000);
	*/
	byte sensorRequest[] = {
		0x2D, // Dirección del dispositivo (0x2D es la representación hexadecimal de 45)
		0x03, // Número de función (Lectura de datos)
		0x00, 0x00, // Dirección de inicio del registro a leer (por ejemplo, 0x0000)
		0x00, 0x03, // Cantidad de registros a leer (2 bytes)
		0x02, 0x67 // CRC
	};
	
	Serial.println("envia solicitud");
	sendInstruction(RS485_EN, SERIAL_RS485, sensorRequest);
	Serial.println("Enviada...Leer");
	byte* response = readCommunication(SERIAL_RS485, 11); // Obtener la respuesta
	for (int i = 0; i < 11; i++) {
		Serial.write(response[i]); // Escribir cada byte del array
	}
	if (response != nullptr) { // Verifica si se recibió una respuesta
		Serial.println("Respuesta recibida:");
		int CO2 = (response[3] << 8) | response[4];
		int Temp = (response[5] << 8) | response[6];
		int Humedad = (response[7] << 8) | response[8];

		Serial.println("");

		Serial.print("CO2: ");
		Serial.print(CO2);
		Serial.println(" ppm");

		Serial.print("Humedad: ");
		Serial.print(Humedad/100);
		Serial.println(" %");

		Serial.print("Temperatura: ");
		Serial.print(Temp/100);
		Serial.println(" C");
		Serial.println(); // Imprime una nueva línea al final
		delete[] response;
	}
	delay(5000);
}

