
Spostare in IsoBase i cicli di base, anche se personalizzati
Corretta vars_inc.cfs (acq_x, ...)
Spostate variabili app in generic_inc.cfs, in modo che vars_inc sia rimanga di base
	Da includere in plasma_inc e/o PGM dei canali che usano tali variabili
G920 rimossi pollici (lasciata personalizzazione)
G888 rimossi pollici
reinstallata versione di base G400x per togliere pollici/mm

Ho modificato la specifica per adattarla a ISOBASE:
GUIDELINE:
-------- 2 --------
I cicli fissi possono necessitare di interventi minori per adattarli
all'applicazione (grafica, verify, ecc..).
NELLA CARTELLA ISOBASE DI OGNI APPLICAZIONE, tenere quindi una cartella _BASE con
i cicli fissi da cui si è partiti per l'adattamento.
Questo è utile per facilitare l'integrazione di modifiche, in modo da poter
facilmente risalire a cosa si è modificato rispetto alla versione di base.

-------- 3 --------
Quando si copiano i cicli nell'applicazione, copiare SEMPRE anche gli eventuali
file vuoti CFS_xxx_yyy.ID, che servono per tenere traccia della versione.

-------- 4 --------

Per utilizzare le versioni 1.21 e successive:

    Richiesta ISO BETA 17.2 (Kvara BETA 32.2) o successive!

    attivare CN_ENAB_G10_EXCLUDE_UNGRP in Opzioni CN
    (Exclude ungrouped axes with G10).
    In caso contrario si verifica un errore alla partenza