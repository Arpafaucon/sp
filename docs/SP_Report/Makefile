all: html

html:
	doxygen conf/Doxyfile

html_pdf: html
	wkhtmltopdf build/html/report.html SP_Roussel_Gregoire.pdf
