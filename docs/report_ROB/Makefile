all: html

html:
	doxygen conf/Doxyfile

html_pdf: html
	wkhtmltopdf build/html/report.html ROB314_Roussel_Gregoire.pdf

latex_pdf: html 
	cd build/latex && make
	cp build/latex/refman.pdf ROB314_Roussel_Gregoire_tx.pdf