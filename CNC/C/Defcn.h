#define         NUM_CANALI             10   // Parametri e registri canali.
#define         NUM_ASSI               90   // Parametri e registri assi.
#define         NUM_ORIG                8   // Numero origini
#define         NUM_CZONE              11   // Struttura Correzione Zone.
#define         NUM_CZONE_ENTRIES     256   // Voci struttura Correzione Zone.
#define         NUM_MAG                 6   // Struttura Magazzino Tools ( Revolver )
#define         NUM_WEAR                5   // Struttura Usura e Vita Utensile.
#define         NUM_NC                 10   // Correttori Utensili tipo system 1
#define         NUM_TWI                16
#define         NUM_THD                10
#define         NUM_CED                50
#define         NUM_TAB                10
#define         NUM_GORG               10
#define         NUM_PROBE               4   // Numero Probe
#define         NUM_TORCIA              4   // Numero Torce Taglio Plasma
#define         NUM_GANTRY              2   // Numero Assi Gantry
#define         NUM_VRTC               12   // Tabelle compensazione vettoriale run-time
#define         NUM_VRTC_TRANS_PTS     32
#define         NUM_VRTC_TRANS_SEN      4
#define         NUM_VRTC_TRANS_KP       4
#define         NUM_LNDC_TRANS          4
#define         NUM_LNDC_TRANS_PTS     32

#define         NUM_VG                128   // Variabili globali ISO
#define         NUM_VL                256   // Variabili locali ISO
#define         NUM_NEST                8   // Numero di nesting ISO
#define         NUM_VA                 32   // Variabili automatiche ISO

#define         NUM_C                 255   // Registri C
#define         NUM_M                 256   // Registri M del PLC
#define         NUM_R                 128   // Registri R del PLC

#define         NUM_COUNTER            30   // Counter del PLC
#define         NUM_PLCERR             32   // Bit Array per allarmi PLC
#define         NUM_PLCMSG             32   // Bit Array per messaggi PLC
#define         NUM_EVPLC            4001   // Buffer log eventi PLC
#define         NUM_AXQTE            4001   // Buffer log quote assi da PLC
#define         NUM_MT                 20   // Registri log PLC a bit
#define         NUM_RT                 20   // Registri log PLC a dword
#define         NUM_ABTR               20   // Registri log PLC su buffer assi

#define         DE_NUM_DRV              1   // (se usato, impostare ad 8) Numero driver per RING D.Electron
#define         DE_NUM_RINGS            1   // (se usato, impostare ad 8) Numero rings D.Electron
#define         NUM_RINGS               1   // Numero rings Sercos / Mechatrolink
#define         NUM_DRV                16   // Numero drives per ring

#define         SYNC_MARKERS           32   // Sincronizzazione multicanale

#define         NUM_IOX               256   // Numero inp. o outp. per redirezione
#define         NUM_IOXDW               8   // Deve essere (NUM_IOX / 32)
#define         NUM_AX_USER_DATA        1   // n di dword dati utente in ax

#define         N_TM_MACHINES           1   // Numero di macchine per TM
#define         N_TM_HEADS              1   // Numero di teste per TM
#define         N_TM_TABLES             1   // Numero di tavole per TM
#define         N_TM_SHUTTERS           8   // Numero di shutter per TM
#define         N_TM_PARTS              1   // Numero di part-program per TM

