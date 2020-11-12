# invoke SourceDir generated makefile for mss_pa.per4f
mss_pa.per4f: .libraries,mss_pa.per4f
.libraries,mss_pa.per4f: package/cfg/mss_pa_per4f.xdl
	$(MAKE) -f C:\ti\Thesis_radar\pa_18xx_mss/src/makefile.libs

clean::
	$(MAKE) -f C:\ti\Thesis_radar\pa_18xx_mss/src/makefile.libs clean

