  .org $8000

  ; LDA #$01
  ; STA $0200
  ; LDA #$05
  ; STA $0201
  ; LDA #$08
  ; STA $0202

;  .org $fffc
;  .word $8000
;  .word $0000

  LDA #$FF
  STA $6002

  LDA #$55
  STA $6000
  LDA #$AA
  STA $6000
  JMP $8005
