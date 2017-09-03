// Do not remove the include below
#include "OSRAM_SLx_2016.h"
#include "Arduino.h"


/*
 *  SLx-2016 display interfacing test for the Arduino Diecimila
 *  Hessel Schut, hessel@isquared.nl
 *  version 0.32
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// address lines
#define a0 52
#define a1 53

// !wr (write, active low)
#define _wr  51
#define d7  5 // too few pins on port C. borrow another

// write a character 'dchar' in position 'digit' on the display
void OSRAM_SLx::writeDigit(int digit, char dchar) {

  // set address, the SLx-2016 defines address 0 as the rightmost digit
  digitalWrite(a0, digit&1);
  digitalWrite(a1, digit&2);

  // initialize write cycle
  digitalWrite(_wr, LOW);

  // write character data
  PORTC = dchar;
  digitalWrite(d7, dchar & B01000000); // port C is 6 bits breed, ascii 7... ;)

  // finish write cycle
  digitalWrite(_wr, HIGH);
}


void OSRAM_SLx::clearDisplay() {
  // The 2016 has a pin for this, but this is more economic on IO pins:
  for (int pos = 0; pos <= 3; pos++) {
    // write a bunch of spaces to the display
    writeDigit(3 - pos, 0x20);
  };
}


// write ascii text to the display
void OSRAM_SLx::print_slx2016(char mesg_string[]) {

  // initialize display buff
  char disp[5] = "    ";

  clearDisplay();

  if (strlen(mesg_string) > 4) {
    // scroll through the message
    for (int pos = 0; pos < strlen(mesg_string); pos++) {
      writeDigit(0, mesg_string[pos]);
      for (int offset = 1; offset <= 3; offset++) {
        if (pos > (offset - 1)) {
          writeDigit(offset, mesg_string[pos - offset]);
        };
      };

      delay(200);
    };
  }
  else {
    // string fits on display, no scrolling

    // assign mesg_str to blank display buffer
    for (int i = 0; i <= strlen(mesg_string) - 1; i++) {
      disp[i] = mesg_string[i];
    };
    // write display buffer to display
    for (int pos = 0; pos <= 3; pos++) {
      writeDigit(3 - pos, disp[pos]);
    };
  };
}

// int to ascii string ('borrowed' from http://www.jb.man.ac.uk/~slowe/cpp/itoa.html)
char* itoa(int val, int base){
  static char buf[32] = {
    0  };

  int i = 30;

  for(; val && i ; --i, val /= base)

    buf[i] = "0123456789abcdef"[val % base];

  return &buf[i+1];
}

// get length of string (iterate until nul terminator)
int strlen(char *str) {
  char *i;
  for (i=str; *i; i++);
  return i-str;
}


  //
  // // define pin modes
  // pinMode(a0, OUTPUT);
  // pinMode(a1, OUTPUT);
  // pinMode(_wr, OUTPUT);
  // pinMode(d7, OUTPUT);
  //
  // // port C (the Arduino analog pins) is used as data bus
  // // set data direction register of this port to output
  // DDRC |= B11111111;
  //
  // // display string on init
  // char test[] = "init";
  //
  // // flash init string
  // for (int i = 0; i <= 10; i++) {
  //
  //   clearDisplay();
  //   delay(50);
  //
  //   for (int pos = 0; pos <= 3; pos++) {
  //     writeDigit(3 - pos, test[pos]);
  //   };
  //   delay(50);
  //
  // };
  //
  // delay(1000);
  // // setup finished, jump to loop()




//
//   print_slx2016("Hello, world!");
//   delay(500);
//
//   print_slx2016("BITE");
//   delay(500);
//
//   print_slx2016("The module has to initialized in order to use it correctly!!");
//    delay(500);
// //
// //  for (int n = 0; n <=9999; n++) {
// //    print_slx2016(itoa(n, 10));
// //    delay(15);
// //  }
// //  delay(500);
