  .org $8000

  LDA #$01
  STA $0200
  LDA #$05
  STA $0201
  LDA #$08
  STA $0202

 .org $fffc
 .word $8000
 .word $0000
