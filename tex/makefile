# Defines the name of the tex file
FILE=report

$(FILE).pdf : $(FILE).tex $(FILE).bib
	pdflatex $(FILE)
	biber $(FILE)
	pdflatex $(FILE)
	pdflatex $(FILE)

clean :
	rm *.aux *.bbl *.bcf *.blg *.lof *.log *.lot *.xml *.toc
