#define NUM_MSG_LEN         64          // Lunghezza stringa
#define NUM_STR_TAB         128         // Numero Stringhe non retentive
#define NUM_STR_TAB_RTN     128         // Numero Stringhe retentive

#define NUM_APP_TCP         100         // Numero registri TCP

#define NUM_APP_EOK         500         // Numero registri EOK
#define NUM_APP_ETK         500         // Nimero registri ETK
#define NUM_APP_EBK         500         // Numero registri EBK

#define NUM_PLCOP           2
#define NUM_COSTK           64
#define NUM_WTJPLCOP        2	        // WTJ Options
#define NUM_WTJCOSTK        64          // WTJ K Constants
#define NUM_LSRPLCOP        2	        // LSR Options
#define NUM_LSRCOSTK        64          // LSR K Constants

#define NUM_APP_gplc        100         // Numero registri plc generali
#define NUM_APP_rplc         50         // Numero registri plc generali (RETAIN)
#define NUM_APP_ggui        100         // Numero registri gui generali
#define NUM_APP_rgui         50         // Numero registri gui generali (RETAIN)
#define NUM_APP_giso        100         // Numero registri iso generali
#define NUM_APP_riso         50         // Numero registri iso generali (RETAIN)

#define NUM_APP_plplc       100         // numero registri plc plasma
#define NUM_APP_wtplc       100         // numero registri plc waterjet
#define NUM_APP_lsplc       100         // numero registri plc laser
#define NUM_APP_oxplc       100         // numero registri plc oxitaglio
#define NUM_APP_rplplc      100         // Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA PLASMA RETAIN)
#define NUM_APP_rwtplc      100         // Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA WATERJET RETAIN)
#define NUM_APP_rlsplc      100         // Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA LASER RETAIN)
#define NUM_APP_roxplc      100         // Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA OXI RETAIN)

#define NUM_APP_plgui       100         // numero registri gui plasma
#define NUM_APP_wtgui       100         // numero registri gui waterjet
#define NUM_APP_lsgui       100         // numero registri gui laser
#define NUM_APP_oxgui       100         // numero registri gui oxitaglio
#define NUM_APP_rplgui      100         // Riservati in scrittura a GUI (GLOBALI TECNOLOGIA PLASMA RETAIN)
#define NUM_APP_rwtgui      100         // Riservati in scrittura a GUI (GLOBALI TECNOLOGIA WATERJET RETAIN)
#define NUM_APP_rlsgui      100         // Riservati in scrittura a GUI (GLOBALI TECNOLOGIA LASER RETAIN)
#define NUM_APP_roxgui      100         // Riservati in scrittura a GUI (GLOBALI TECNOLOGIA OXI RETAIN)

#define NUM_APP_pliso       100         // numero registri iso plasma
#define NUM_APP_wtiso       100         // numero registri iso waterjet
#define NUM_APP_lsiso       100         // numero registri iso laser
#define NUM_APP_oxiso       100         // numero registri iso oxitaglio
#define NUM_APP_rpliso      100         // Riservati in scrittura per ISO (GLOBALI TECNOLOGIA PLASMA RETAIN)
#define NUM_APP_rwtiso      100         // Riservati in scrittura per ISO (GLOBALI TECNOLOGIA WATERJET RETAIN)
#define NUM_APP_rlsiso      100         // Riservati in scrittura per ISO (GLOBALI TECNOLOGIA LASER RETAIN)
#define NUM_APP_roxiso      100         // Riservati in scrittura per ISO (GLOBALI TECNOLOGIA OXI RETAIN)

#define MAX_POWDEL_SPLINES  220         // n of powdel splines
#define MAX_POWDEL_SPANS    23          // n of powdel spline polynomial spans
#define MAX_POWDEL_POINTS   24          // n of spline points (MAX_POWDEL_SPANS+1)

#define NUM_APP_EDK         100         // Numero registri EDK

// PLC TABLE BASED INTERPOLATION
#define NUM_PLC_TBI_DIVS    512         // Numero di entry per tabella
#define NUM_PLC_TBI         4           // Numero di tabelle