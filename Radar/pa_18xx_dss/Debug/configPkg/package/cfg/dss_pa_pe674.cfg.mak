# invoke SourceDir generated makefile for dss_pa.pe674
dss_pa.pe674: .libraries,dss_pa.pe674
.libraries,dss_pa.pe674: package/cfg/dss_pa_pe674.xdl
	$(MAKE) -f C:\ti\Thesis_radar\pa_18xx_dss/src/makefile.libs

clean::
	$(MAKE) -f C:\ti\Thesis_radar\pa_18xx_dss/src/makefile.libs clean

