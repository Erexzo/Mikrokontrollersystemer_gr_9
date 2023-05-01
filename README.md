Gruppe nr 9
IELET2111-oppgave3-i2c_klient
Oppgavebeskrivelse:

Oppgave 3 - I2C slave for monitorering av spenning, temperatur og viftehastighet via lys-detektor. Et system knyttet til viftekontrolleren i oppgave 2. Det skal monitorere spenning, temperatur og viftehastighet (ved hjelp av lys-detektor). Estimert utfordringsgrad: 2

    MÅ: Lage en I2C slave, enten egenkodet eller via rammeverk.
    MÅ: Noden skal overvåke både egen og ekstern spenning
    MÅ: Noden skal overvåke temperatur via en termistor.
    MÅ: Noden skal bed hjelp av lys- og fotodiode bestemme hastigheten til ekstern vifte.
    MÅ: Via feilutganger vil noden varsle både høy temperatur, og gi feil dersom begge viftene står stille i for lang tid, eller temperaturen er for høy.
    MÅ: Endringer i innstillinger må kunne gjøres over I2C og lagres på kontrolleren.
    BØR: Viften skal overvåkes over tid, slik at avvik i viftehastighet og evt varsler kan leses av via I2C.
    Tilleggskretser kan brukes etter behov. EEPROM / ADC / OPAMP osv.
