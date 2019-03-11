all: config web pdf

config:
	doxygen conf/Doxyfile

web: config
	wkhtmltopdf build/html/report.html ROB314_Roussel_Gregoire.pdf

pdf: config 
	cd build/latex && make
	cp build/latex/refman.pdf ROB314_Roussel_Gregoire_tx.pdf