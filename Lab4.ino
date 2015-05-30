/*

Run this sketch on two Zigduinos, open the serial monitor at 9600 baud, and type in stuff
Watch the Rx Zigduino output what you've input into the serial port of the Tx Zigduino

*/

#include <ZigduinoRadio.h>

#define NODE_ID 0x0001
#define DST_ID 0x0008
#define SRC_ID 0x0001

// node id of this node. change it with different boards
#define CHANNEL 22      // check correspond frequency in SpectrumAnalyzer
#define TX_TRY_TIMES 30
//5  // if TX_RETRY is set, pkt_Tx() will try x times before success
#define TX_DO_CARRIER_SENSE 1
#define TX_SOFT_ACK 1   // only affect RX part(send ACK by hw/sw). TX still check ACK by  hardware in this code. modify libraries if necessary.
#define TX_SOFT_FCS 0
#define TX_RETRY 1      // pkt_Tx() retransmit packets if failed.
#define TX_BACKOFF 100  // sleep time in ms
#define TX_HEADER_LEN 9

#define TIMEOUT 10000

uint8_t TxBuffer[128]; // can be used as header and full pkt.
uint8_t RxBuffer[128];
uint8_t softACK[8];
char teststr[12] = "ABCDEFGHIJK";
char backstr[12] = "TT";
char pingstr[12] = "TT";
char temp[12] = "TT";


uint8_t broadcast = 0 ;
uint8_t nextNode = 0;
int count = 0;
uint32_t totalRTT = 0;
uint8_t fcs_failed = 0;

uint8_t TX_available; // set to 1 if need a packet delivery, and use need_TX() to check  its value
// here are internal variables, please do not modify them.
uint8_t retry_c;
uint8_t RX_available; // use has_RX() to check its value
uint8_t RX_pkt_len;

int mode = 0 ;
uint8_t pingidx = 0;
uint8_t successful = 0;
unsigned long ts1, ts2, tnow;
unsigned long tt[101];
uint8_t prenxt[2]; // previous and next node (src to dst) & (dst to src)
// the setup() function is called when Zigduino staets or reset
uint8_t threshold;

void setup() {
	prenxt[0] = 0;
	prenxt[1] = 0;

	teststr[0] = '0';
	teststr[1] = '0' + DST_ID;
	teststr[2] = 'x';
	teststr[3] = '0' + NODE_ID;
	teststr[4] = '\0';
	init_header();
	retry_c = 0;

	if (NODE_ID == 0x0001) {
		TX_available = 1;
	} else {
		TX_available = 0;
	}

	RX_available = 0;
	ZigduinoRadio.begin(CHANNEL, TxBuffer);
	ZigduinoRadio.setParam(phyPanId, (uint16_t)0xABCD );
	ZigduinoRadio.setParam(phyShortAddr, (uint16_t)NODE_ID );
	ZigduinoRadio.setParam(phyCCAMode, (uint8_t)3);
	Serial.begin(9600);

	// register event handlers
	ZigduinoRadio.attachError(errHandle);
	ZigduinoRadio.attachTxDone(onXmitDone);
	ZigduinoRadio.attachReceiveFrame(pkt_Rx);
	//ts1 = millis();

}

// the function is always running
void loop() {
	uint8_t inbyte;
	uint8_t inhigh;
	uint8_t inlow;
	uint8_t tx_suc;

	uint8_t pktmd; // packet mode
	uint8_t drop = 0;


	//testttttttttttagain

	if (has_RX() && (!fcs_failed)) {
		pktmd = RxBuffer[TX_HEADER_LEN] - '0';
		if ((pktmd - mode) == 1 && pktmd <= 2) {
			mode = pktmd;
			drop = 0;
		} // change mode
		else if (pktmd == mode) {
			drop = 0;
		} else { // already has path , drop must = 1
			drop = 1;
			TX_available = 0;
		}

		cleanRx();

		/*
		Serial.println("After clean Rx Buffer " ) ;


		for (uint8_t i = 9 ; i < 20 ; i++) {
		  Serial.print(" Rx[ ");
		  Serial.print(i);
		  Serial.print(" ] = ");
		  Serial.print(RxBuffer[i]-'0');
		  Serial.print(" , ");
		} // print packet

		Serial.println(" ");
		*/

		if (!drop) {

			switch (mode) {
			case 0 :
				if (NODE_ID == SRC_ID) {
					TX_available = 0; //
					broadcast = 0; //
				} else if (NODE_ID == DST_ID) { //start back
					reversepath();
					for (int j = 12; j >= 1; j--) //shift temp to insert dst
						temp[j] = temp[j - 1];

					strcpy(backstr, temp);
					backstr[0] = '1';
					backstr[1] = '1';
					backstr[2] = 'x';
					backstr[3] = DST_ID + '0';

					TX_available = 1;
					mode = 1 ;

					fillRoutingTbl();
					nextNode = backstr[4] - '0';

					Serial.println("start bbbbbbbbbbbbbbbbbbbbbbbbbbbbb") ;

				} else { //middle
					uint8_t i = TX_HEADER_LEN + 3;
					uint8_t already_in_path = 0;
					while (RxBuffer[i] != '\0') {
						if (RxBuffer[i] == ('0' + NODE_ID)) {
							already_in_path = 1;
						}
						i++;
					} // check if in list

					if (already_in_path) {
						TX_available = 0;
						broadcast = 0;
					} // already_in_path
					else {
						strncpy(&teststr[0], (char*)&RxBuffer[TX_HEADER_LEN], i - TX_HEADER_LEN);
						/*
						for(uint8_t j = 0 ; j< i-TX_HEADER_LEN ; j++){
						  teststr[j] = RxBuffer[TX_HEADER_LEN+j];
						}
						*/
						i -= TX_HEADER_LEN ;
						teststr[i] = NODE_ID + '0';
						teststr[i + 1] = '\0';
						TX_available = 1;
						broadcast = 10;

						// Serial.print("teststr: ");
						// Serial.println(teststr);

					} // not in path
				} // middle
				break ;

			case 1 :
				if (NODE_ID == SRC_ID) {

					reversepath();
					strcpy(pingstr, temp);
					pingidx = 1; // start to ping
					pingstr[0] = '2';
					pingstr[1] = '0' + DST_ID;
					pingstr[2] = pingidx;
					pingstr[3] = SRC_ID + '0' ;
					// nextNode = pingstr[4] - '0';

					uint8_t k = 3 ;
					uint8_t flg = 0 ;

					Serial.print("~~~");
					while (1) {
						while (pingstr[k] != '\0' && flg == 0) {
							Serial.print(pingstr[k]);
							k++;
						}
						flg = 1;
					}

					mode = 2;
					TX_available = 1;

					fillRoutingTbl();
					nextNode = prenxt[0] - '0';

					Serial.println(" Start pingggggggggggggggggggggggggggggggggggggg")  ;

					/*
					for (uint8_t i = 9 ; i < 20 ; i++) {
					  Serial.print(" Rx[ ");
					  Serial.print(i);
					  Serial.print(" ] = ");
					  Serial.print(RxBuffer[i] - '0');
					  Serial.print(" , ");
					}
					Serial.println(" ");


					                for (uint8_t i = 0 ; i < 12 ; i++){
					                      Serial.print(" pingstr[ ");
					                      Serial.print(i);
					                      Serial.print(" ] = ");
					                      Serial.print(pingstr[i]-'0');
					                      Serial.print(" , ");
					                }
					                Serial.println(" ");
					*/


				} // src
				else if (NODE_ID == DST_ID) {
					TX_available = 0;
				} // dst
				else {
					uint8_t i = TX_HEADER_LEN + 3;
					while (RxBuffer[i] != (NODE_ID + '0')) {
						i++;
					}

					i = TX_HEADER_LEN + 3;
					while (RxBuffer[i] != '\0') {
						i++;
					}
					strncpy(&backstr[0], (char*)&RxBuffer[TX_HEADER_LEN], i - TX_HEADER_LEN);
					TX_available = 1 ;

					fillRoutingTbl();
					nextNode = RxBuffer[i + 1] - '0';

					Serial.print("backstr :");
					Serial.println(backstr);

				} // middle

				Serial.print("nextNode:::::::::");
				Serial.println(nextNode);

				break ;

			case 2 :

				Serial.print("backstr :");
				Serial.println(backstr);
				Serial.println("Rx case 2  stopppppppp ");

				if (NODE_ID == SRC_ID) {


					if (RxBuffer[TX_HEADER_LEN + 2] == 'x') {
						TX_available = 0;
						broadcast = 0;
					} // drop packet
					else {
						tnow = millis();
						if ((tnow - tt[RxBuffer[TX_HEADER_LEN + 2]]) <= TIMEOUT) {
							totalRTT += (tnow - tt[RxBuffer[TX_HEADER_LEN + 2] - '0']);
							successful++;
							Serial.print("successssssssssss : ");
							Serial.println(successful);
						} // no timeout
					}
				} // src
				else if (NODE_ID == DST_ID) {
					reversepath();
					strcpy(pingstr, temp);
					pingstr[0] = '2';
					pingstr[1] = '1';
					pingstr[2] = RxBuffer[TX_HEADER_LEN + 2];

					// nextNode = pingstr[4] - '0';
					nextNode = prenxt[1] - '0';

					TX_available = 1;
				} // dst
				else {
					uint8_t i = TX_HEADER_LEN + 3;
					while (RxBuffer[i] != NODE_ID + '0') {
						i++;
					}

					// nextNode = RxBuffer[i+1] - '0';

					TX_available = 1;
					i = TX_HEADER_LEN + 3;
					while (RxBuffer[i] != '\0') {
						i++;
					}
					strncpy(&pingstr[0], (char*)&RxBuffer[TX_HEADER_LEN], i - TX_HEADER_LEN);

					if (RxBuffer[TX_HEADER_LEN + 1] == (DST_ID + '0')) {
						nextNode = prenxt[0] - '0';
					} // src 2 dst
					else if (RxBuffer[TX_HEADER_LEN + 1] == (SRC_ID + '0')) {
						nextNode = prenxt[1] - '0';
					} // dst 2 src
					else {
						// ??????
						Serial.println("WTF nextNode ???");
					}


				} // middle

				break ;

			default :
				Serial.println("default!");
			} // switch

		} // not drop

		/*
		Serial.println();
		Serial.print("Rx: ");
		for (uint8_t i = 0; i < RX_pkt_len; i++) {
		  inbyte = RxBuffer[i];
		  if (printable(inbyte)) {
		    Serial.write(inbyte);
		  } else {
		    Serial.print(".");
		  }
		} // print packet
		*/
		// Serial.print(" pingIndex : ");
		// Serial.println(RxBuffer[11]);


		/*
		        Serial.println();
		        Serial.print("LQI: ");
		        Serial.print(ZigduinoRadio.getLqi(), 10);
		        Serial.print(", RSSI: ");
		        Serial.print(ZigduinoRadio.getLastRssi(), 10);
		        Serial.print(" dBm, ED: ");
		        Serial.print(ZigduinoRadio.getLastEd(), 10);
		        Serial.println("dBm");
		*/

	} // if has_RX

	if (NODE_ID == DST_ID && mode == 1) {
		TX_available = 1;
	}

	if (pingidx > 0 && pingidx <= 100) {
		Serial.print("pingidx:");
		Serial.print(pingidx);
		Serial.println("");
		TX_available = 1;
	} else if (pingidx > 100) {
		TX_available = 0;
		tnow = millis();
		if (tnow - tt[100] > TIMEOUT) {
			Serial.print("totalRTT:");
			Serial.println(totalRTT);
			Serial.print("Average totalRTT: ");
			Serial.println(totalRTT / successful) ;
			Serial.print("successful:");
			Serial.print(successful);
			Serial.println("%");
			while (1) {} // infinite loop
		} // game over
	} // ping index > 100

	if (need_TX()) {
		delay(TX_BACKOFF);

		switch (mode) {
		case 0 :
			Serial.println("Ready for broadcast!!");

			//teststr[3] = 'a' ;

			/*
			for (int q = 0 ; q < 6 ; q++){
			             Serial.print(" teststr[ ");
			             Serial.print(q);
			             Serial.print(" ] = ");
			             Serial.print(teststr[q]);
			             Serial.print(" , ");
			}
			       Serial.println(" ");
			*/

			tx_suc = pkt_Tx(0xffff, teststr);
			Serial.println(teststr);
			break ;

		case 1 :
			Serial.println("Ready for back!!") ;
			/*
			for (uint8_t i = 0 ; i < 12 ; i++){
			             Serial.print(" backstr[ ");
			             Serial.print(i);
			             Serial.print(" ] = ");
			             Serial.print(backstr[i]-'0');
			             Serial.print(" , ");
			       }
			       Serial.println(" ");
			*/
			tx_suc = pkt_Tx(nextNode, backstr);
			break ;

		case 2 :
			Serial.println("Tx case 2");

			if (NODE_ID == SRC_ID) {
				tt[pingidx] = millis();
				pingstr[2] = pingidx;
				pingidx++;
			}

			/*
			          Serial.println("pingstr before senddddddddddd");
			                for (uint8_t i = 0 ; i < 12 ; i++){
			                      Serial.print(" pingstr[ ");
			                      Serial.print(i);
			                      Serial.print(" ] = ");
			                      Serial.print(pingstr[i]-'0');
			                      Serial.print(" , ");
			                }
			          Serial.println(" ");
			*/

			tx_suc = pkt_Tx(nextNode, pingstr);
			break ;

		default :
			Serial.println("default!");
		} // switch mode

		if (--broadcast) {
			TX_available = 1;
		}
	} // if need_TX

	delay(500);
} // loop

void init_header() {
	if (TX_SOFT_ACK) {
		TxBuffer[0] = 0x61; // ack required
	} else {
		TxBuffer[0] = 0x41; // no ack required
	}
	TxBuffer[1] = 0x88;
	TxBuffer[2] = 0;    // seqence number
	TxBuffer[3] = 0xCD;
	TxBuffer[4] = 0xAB; //Pan Id
	TxBuffer[5] = 0x01; //dest address low byte
	TxBuffer[6] = 0x00; //dest address hight byte
	TxBuffer[7] = NODE_ID & 0xff; //source address low byte
	TxBuffer[8] = NODE_ID >> 8; //source address hight byre
	softACK[0] = 0x42;
	softACK[1] = 0x88;
}

/* the packet transfer function
 * parameters :
 *     dst_addr : destnation address, set 0xffff for broadcast. Pkt will be dropped if  NODE_ID != dst addr in packet header.
 *     msg : a null-terminated string to be sent. This function won't send the '\0'. Pkt  header will be paded in this function.
 * return values : this function returns 0 if success, not 0 if problems (no ACK after  some retries).
 *
 * This function set the headers to original msg then transmit it according your policy  setting.
 * The most important function is ZigduinoRadio.txFrame(TxBuffer, pkt_len) which transmit  pkt_len bytes from TxBuffer, where TxBuffer includes header.
 * Note that onXmitDone(radio_tx_done_t x) is right called after txFrame(), you can do  some status checking at there.
 *
 * Feel free to modify this function if needed.
 */
uint8_t pkt_Tx(uint16_t dst_addr, char* msg) {
	uint16_t fcs;
	uint8_t i;
	uint8_t pkt_len;
	uint8_t tmp_byte;
	radio_cca_t cca = RADIO_CCA_FREE;
	int8_t rssi;

	// process the dst addr, 0xffff for broadcast
	TxBuffer[5] = dst_addr & 0x00ff;
	TxBuffer[6] = dst_addr >> 8;
	tmp_byte = TxBuffer[0];

	if (dst_addr == 0xffff) { // broadcast, no ACK required
		TxBuffer[0] = 0x41;
	}
	// fill the payload
	for (i = 0; msg[i] != '\0'; i++) {
		TxBuffer[TX_HEADER_LEN + i] = msg[i];
	}
	pkt_len = TX_HEADER_LEN + i;

	//uint8_t msg_len = i;  //for CRC

	// fill the software fcs
	if (TX_SOFT_FCS) {
		fcs = cal_fcs(TxBuffer, pkt_len);
		//fcs = cal_crc(TxBuffer, msg_len, TX_HEADER_LEN);  //for CRC

		TxBuffer[pkt_len++] = fcs & 0xff;
		TxBuffer[pkt_len++] = fcs >> 8;
	}
	// hardware fcs, no use
	pkt_len += 2;
	// transmit the packet
	// retry_c will be set to RETRY_TIMES by onXmitDone() if packet send successfully
	if (TX_RETRY) {
		for (retry_c = 0; retry_c < TX_TRY_TIMES; retry_c++) {
			if (TX_DO_CARRIER_SENSE) {
//        cca = ZigduinoRadio.doCca();
				rssi = ZigduinoRadio.getRssiNow();
				threshold = rssi - RSSI_BASE_VAL;
				Serial.print("Threshold = ");
				Serial.println(threshold);
//      if(cca == RADIO_CCA_FREE)
//      if(rssi == -91){
				if (threshold <= 10 || threshold == 255) {
					ZigduinoRadio.txFrame(TxBuffer, pkt_len);
				} else {
					Serial.print("ca fail with rssi = ");
					Serial.println(rssi);
				}
			} else {
				ZigduinoRadio.txFrame(TxBuffer, pkt_len);
			}
			delay(TX_BACKOFF);
		}
		retry_c--; // extra 1 by for loop, if tx success retry_c == TX_TRY_TIMES
	} else {
		if (TX_DO_CARRIER_SENSE) {
//      cca = ZigduinoRadio.doCca();
			rssi = ZigduinoRadio.getRssiNow();
			threshold = rssi - RSSI_BASE_VAL;
//      if(cca == RADIO_CCA_FREE)
//      if(rssi == -91){
			if (threshold <= 10 || threshold == 255) {
				ZigduinoRadio.txFrame(TxBuffer, pkt_len);
			} else {
				Serial.print("ca fail with rssi = ");
				Serial.println(rssi);
			}
		} else {
			ZigduinoRadio.txFrame(TxBuffer, pkt_len);
		}
	}
	TxBuffer[0] = tmp_byte;
	return retry_c == TX_TRY_TIMES;
}

/* the event handler which is called when Zigduino got a new packet
 * don't call this function yourself
 * do sanity checks in this function, and set RX_available to 1 at the end
 * the crc_fail parameter is a fake, please ignore it
 */
uint8_t* pkt_Rx(uint8_t len, uint8_t* frm, uint8_t lqi, uint8_t crc_fail) {
	uint16_t fcs;
	// This function set RX_available = 1 at the end of this function.
	// You can use has_RX() to check if has packet received.

	// Software packet filter :
	// Check pkt_len, dst_addr, FCS. Drop pkt if fails
	if (len < TX_HEADER_LEN) {
		return RxBuffer;
	}
	// keep the pkt only if broadcast or dst==me
	if ( (frm[5] != NODE_ID & 0xff) || (frm[6] != NODE_ID >> 8) ) {
		if (frm[5] != 0xff || frm[6] != 0xff) {
			return RxBuffer;
		}
	}
	// check fcs first, drop pkt if failed
	if (TX_SOFT_FCS) {
		fcs = cal_fcs(frm, len - 2);
		//fcs = cal_crc(frm, len-2, TX_HEADER_LEN); //for CRC
		if (fcs != 0x0000) { //for CRC fcs sholud == 0
			return RxBuffer;
		}
	}
	// send software ack
	/* if(frm[0] & 0x20){
	   softACK[2] = frm[2];
	   ZigduinoRadio.txFrame(softACK, 5);
	 }*/
	// now all checks are passed, copy out the received packet
	for (uint8_t i = 0; i < len; i++) {
		RxBuffer[i] = frm[i];
	}
	RX_pkt_len = len;
	RX_available = 1;
	return RxBuffer;
}

// this function returns TX_available and reset it to 0
uint8_t need_TX() {
	if (TX_available) {
		TX_available = 0;
		return 1;
	}
	return 0;
}


// this function returns RX_available and reset it to 0
uint8_t has_RX() {
	if (RX_available) {
		RX_available = 0;
		return 1;
	}
	return 0;
}

// calculate error detecting code
// choose an algorithm for it
uint16_t cal_fcs(uint8_t* frm, uint8_t len) {
	uint16_t fcs = frm[0];
	for (uint8_t i = 1; i < len; i += 1) {
		fcs ^= frm[i];
	}
	return fcs;
}

//==================================CRC Computation==================
const uint8_t CRC7_POLY = 0x91;

uint16_t cal_crc(uint8_t* frm, uint8_t length, uint8_t HEADER_LEN) {
	uint8_t i, j, crc = 0;

	for (i = 0; i < length; i++) {
		crc ^= frm[HEADER_LEN + i];
		for (j = 0; j < 8; j++) {
			if (crc & 1)
				crc ^= CRC7_POLY;
			crc >>= 1;
		}
	}
	return crc;
}

//===================================================================

uint8_t printable(uint8_t in) {
	if (32 <= in && in <= 126) {
		return 1;
	}
	return 0;
}

void errHandle(radio_error_t err) {
	Serial.println();
	Serial.print("Error: ");
	Serial.print((uint8_t)err, 10);
	Serial.println();
}

// this function is called after the packet transmit function
void onXmitDone(radio_tx_done_t x) {
	Serial.println();
	Serial.print("TxDone: ");
	Serial.print((uint8_t)x, 10);
	if (x == TX_NO_ACK) {
		Serial.print(" NO ACK ");
	} else if (x == TX_OK) {
		Serial.print("(OK)");
		retry_c = TX_TRY_TIMES;
	} else if (x == TX_CCA_FAIL) { // not implemented
		Serial.print("(CS busy)");
	}
	Serial.println();
}

void reversepath() {
	uint8_t pl = TX_HEADER_LEN + 3;
	temp[0] = '0';
	temp[1] = '0';
	temp[2] = '0';
	temp[3] = '0';

	while (RxBuffer[pl] != '\0') {
		pl++;
	} // find path len
	pl -= TX_HEADER_LEN ;

	for (int i = 3; i < pl; i++) {
		temp[i] = RxBuffer[pl  + TX_HEADER_LEN - i + 2];
	}
	temp[pl] = '\0';
}


void cleanRx() {
	uint8_t jump = TX_HEADER_LEN + 3;

	while (RxBuffer[jump] != '\0') {
		jump++;
	} // find path len

	uint8_t j = 0;
	for (uint8_t i = TX_HEADER_LEN + 3 ; i <= jump ; i++ ) {
		RxBuffer[i - j] = RxBuffer[i];
		if (RxBuffer[i] < '1' || RxBuffer[i] > '8') {
			j++;
		}
	}

}
void fillRoutingTbl() {
	uint8_t i = 3;
	while (backstr[i] != (NODE_ID + '0')) {
		i++;
	}
	if (NODE_ID == DST_ID) {
		prenxt[1] = backstr[4];
	} else if (NODE_ID == SRC_ID) {
		prenxt[0] = pingstr[4];
	} else {
		prenxt[0] = backstr[i - 1];
		prenxt[1] = backstr[i + 1];
	}
	Serial.print("Routing table:[");
	Serial.print(prenxt[0]);
	Serial.print(", ");
	Serial.print(prenxt[1]);
	Serial.println("]");
}
