all: web pdf

web:
	doxygen conf/Doxyfile

pdf: web
	wkhtmltopdf build/html/report.html ROB314_Roussel_Gregoire.pdf

