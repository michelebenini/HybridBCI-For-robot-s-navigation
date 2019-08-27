AM_CXXFLAGS = $(CXXFLAGS) -I$(top_srcdir)/src -I/usr/include/eigen3 -std=c++0x

bin_PROGRAMS = hybrid_p300_processing hybrid_smr_processing

hybrid_p300_processing_SOURCES = hybrid_p300_processing.cpp 
hybrid_p300_processing_LDADD   = $(top_builddir)/src/libwtkapp.la \
			     -lwtkcore -lwtkxml -lwtkbci -lwtkdraw -lwtknet -lwtkprotocol -lwtkprocessing\
			     -ldrawtk -lcnbicore -lcnbiloop -ltobicore -ltobiid -ltobiic

hybrid_smr_processing_SOURCES = hybrid_smr_processing.cpp 
hybrid_smr_processing_LDADD   = $(top_builddir)/src/libwtkapp.la \
			     -lwtkcore -lwtkxml -lwtkbci -lwtkdraw -lwtknet -lwtkprotocol -lwtkprocessing\
			     -ldrawtk -lcnbicore -lcnbiloop -ltobicore -ltobiid -ltobiic

