//

/*
$GNRMC,165248.00,A,5048.14814,N,00000.00158,E,0.078,,170419,,,A,V*10
$GNVTG,,T,,M,0.078,N,0.144,K,A*33
$GNGGA,165248.00,5048.14814,N,00000.00158,E,1,12,0.85,57.0,M,45.2,M,,*7C
$GNGSA,A,3,27,16,21,20,26,29,,,,,,,1.49,0.85,1.22,1*0D
$GNGSA,A,3,76,77,78,87,86,70,71,,,,,,1.49,0.85,1.22,2*0C
$GNGSA,A,3,,,,,,,,,,,,,1.49,0.85,1.22,3*03
$GNGSA,A,3,,,,,,,,,,,,,1.49,0.85,1.22,4*04
$GPGSV,3,1,11,05,03,021,,07,06,334,20,08,04,268,,10,06,157,17,1*66
$GPGSV,3,2,11,16,65,295,28,20,24,139,32,21,68,087,34,26,75,187,14,1*63
$GPGSV,3,3,11,27,36,271,11,29,13,080,21,31,11,195,,1*5D
$GPGSV,3,1,11,05,03,021,,07,06,334,,08,04,268,,10,06,157,,6*65
$GPGSV,3,2,11,16,65,295,,20,24,139,,21,68,087,,26,75,187,20,6*6F
$GPGSV,3,3,11,27,36,271,23,29,13,080,24,31,11,195,,6*5E
$GLGSV,3,1,10,69,04,350,,70,19,042,26,71,11,091,33,76,28,178,31,1*72
$GLGSV,3,2,10,77,66,269,11,78,25,326,08,85,01,057,,86,54,041,31,1*74
$GLGSV,3,3,10,87,57,260,25,88,12,244,,1*76
$GLGSV,3,1,10,69,04,350,,70,19,042,,71,11,091,29,76,28,178,,3*7D
$GLGSV,3,2,10,77,66,269,,78,25,326,23,85,01,057,,86,54,041,13,3*7F
$GLGSV,3,3,10,87,57,260,19,88,12,244,,3*7B
$GAGSV,1,1,00,*44
$GAGSV,1,1,00,*44
$GBGSV,1,1,00,*47
$GNGLL,5048.14814,N,00000.00158,E,165248.00,A,A*76
*/

#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include "../../common/gps.h"
#include "../../common/NMEA_parser.h"
#include <gtk/gtk.h>
#include "osm-gps-map.h"

using namespace std;

/*
map sources:
OSM_GPS_MAP_SOURCE_NULL	 
OSM_GPS_MAP_SOURCE_OPENSTREETMAP 	 
OSM_GPS_MAP_SOURCE_OPENSTREETMAP_RENDERER	 
OSM_GPS_MAP_SOURCE_OPENAERIALMAP	 
OSM_GPS_MAP_SOURCE_MAPS_FOR_FREE	 
OSM_GPS_MAP_SOURCE_OPENCYCLEMAP	 
OSM_GPS_MAP_SOURCE_OSM_PUBLIC_TRANSPORT	 
OSM_GPS_MAP_SOURCE_GOOGLE_STREET	 
OSM_GPS_MAP_SOURCE_GOOGLE_SATELLITE	 
OSM_GPS_MAP_SOURCE_GOOGLE_HYBRID	 
OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_STREET	 
OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_SATELLITE	 
OSM_GPS_MAP_SOURCE_VIRTUAL_EARTH_HYBRID 
OSM_GPS_MAP_SOURCE_OSMC_TRAILS
OSM_GPS_MAP_SOURCE_LAST
*/


// all interaction outside the gui has to be handled in these timers... yuck
struct MapAndGps {
    OsmGpsMap *map;
    Gps *gps;

    MapAndGps(OsmGpsMap *m, Gps *g) {
        map = m;
        gps = g;
    }
};

gboolean on_timeout (gpointer user_data) {
    MapAndGps *mgp = static_cast<MapAndGps*>(user_data);

    units::angle::degree_t lat;
    units::angle::degree_t lon;
    units::length::meter_t alt;  

    // get position from gps
    mgp->gps->getPosition(lat,lon, alt);
    
    // update the map
    osm_gps_map_set_center_and_zoom (mgp->map,
                                     lat.value(),
                                     lon.value(),
                                     70);
                                    
  
    return G_SOURCE_CONTINUE; /* or G_SOURCE_REMOVE when you want to stop */ 
}

int main(int argc, char** argv)
{ 
    GtkBuilder *builder;
    GtkWidget *widget;
    OsmGpsMap *map;
    OsmGpsMapLayer *osd;

    // options
    static OsmGpsMapSource_t opt_map_provider = OSM_GPS_MAP_SOURCE_GOOGLE_SATELLITE;
    static gboolean opt_friendly_cache = FALSE;
    static gboolean opt_no_cache = FALSE;
    static char *opt_cache_base_dir = NULL;
    
    
    char *cachedir, *cachebasedir;
    GError *error = NULL;
    GOptionContext *context;

    gtk_init (&argc, &argv);

    context = g_option_context_new ("- Map browser");
    g_option_context_set_help_enabled(context, FALSE);
    cachebasedir = osm_gps_map_get_default_cache_directory();

    if (opt_cache_base_dir && g_file_test(opt_cache_base_dir, G_FILE_TEST_IS_DIR)) {
        cachedir = g_strdup(OSM_GPS_MAP_CACHE_AUTO);
        cachebasedir = g_strdup(opt_cache_base_dir);
    } else if (opt_friendly_cache) {
        cachedir = g_strdup(OSM_GPS_MAP_CACHE_FRIENDLY);
    } else if (opt_no_cache) {
        cachedir = g_strdup(OSM_GPS_MAP_CACHE_DISABLED);
    } else {
        cachedir = g_strdup(OSM_GPS_MAP_CACHE_AUTO);
    }
    
    map = (OsmGpsMap*) g_object_new (OSM_TYPE_GPS_MAP,
                        "map-source",opt_map_provider,
                        "tile-cache",cachedir,
                        "tile-cache-base", cachebasedir,
                        "proxy-uri",g_getenv("http_proxy"),
                        "drag-limit",1,
                        "auto-download", TRUE,
                        "gps-track-point-radius", 1,
                        NULL);

    osd = (OsmGpsMapLayer*) g_object_new (OSM_TYPE_GPS_MAP_OSD,
                        "show-scale",TRUE,
                        "show-coordinates",TRUE,
                        "show-crosshair",TRUE,
                        "show-dpad",TRUE,
                        "show-zoom",TRUE,
                        "show-gps-in-dpad",TRUE,
                        "show-gps-in-zoom",TRUE,
                        "dpad-radius", 30,
                        NULL);

    
    

    


    osm_gps_map_layer_add(OSM_GPS_MAP(map), osd);
    g_object_unref(G_OBJECT(osd));

    builder = gtk_builder_new();

    //Enable keyboard navigation
    osm_gps_map_set_keyboard_shortcut(map, OSM_GPS_MAP_KEY_FULLSCREEN, GDK_KEY_F11);
    osm_gps_map_set_keyboard_shortcut(map, OSM_GPS_MAP_KEY_UP, GDK_KEY_Up);
    osm_gps_map_set_keyboard_shortcut(map, OSM_GPS_MAP_KEY_DOWN, GDK_KEY_Down);
    osm_gps_map_set_keyboard_shortcut(map, OSM_GPS_MAP_KEY_LEFT, GDK_KEY_Left);
    osm_gps_map_set_keyboard_shortcut(map, OSM_GPS_MAP_KEY_RIGHT, GDK_KEY_Right);

    //Build the UI  
    builder = gtk_builder_new();
    gtk_builder_add_from_file (builder, "mapviewer.ui", &error);
    if (error)
        g_error ("ERROR: %s\n", error->message);

    gtk_box_pack_start (
                GTK_BOX(gtk_builder_get_object(builder, "map_box")),
                GTK_WIDGET(map), TRUE, TRUE, 0);

    widget = GTK_WIDGET(gtk_builder_get_object(builder, "window1"));
    gtk_widget_show_all (widget);

    Gps gps;
    units::angle::degree_t lat;
    units::angle::degree_t lon;
    units::length::meter_t alt;  

    gps.getPosition(lat,lon, alt);

    osm_gps_map_set_center_and_zoom (map,
                                     50.8636,
                                     -0.096366,
                                     70);

    // add updater function

    MapAndGps *mgps = new MapAndGps(map, &gps);
    g_timeout_add (1000, on_timeout, mgps);
    gtk_main();
    delete mgps;
    
    
    

    return 0;

}
