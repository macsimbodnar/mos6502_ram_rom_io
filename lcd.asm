PORTB = $6000
PORTA = $6001
DDRB = $6002
DDRA = $6003

  .org $8000

reset:
  LDA #$FF              ; Set all pins on port B to out
  STA DDRB

  LDA #%00000111        ; Set the last 3 pins on port A to output
  STA DDRA

  LDA #$55              ; Set the pattern 01010101 to the port B
  STA PORTB

  LDA #%00000101        ; Set the pattern 00000101 to the port A
  STA PORTA

loop:
  ROR
  STA PORTB

  STA PORTA

  JMP loop