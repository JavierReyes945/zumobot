# Defines the name of the tex file
FILE=report

$(FILE).pdf : $(FILE).tex $(FILE).bib
	pdflatex $(FILE)
	biber $(FILE)
	pdflatex $(FILE)
	pdflatex $(FILE)

clean :
	find . \( -name "*.aux" -o -name "*.bbl" -o -name "*.bcf" -o -name "*.blg" -o -name "*.lof" -o -name "*.log" -o -name "*.lot" -o -name "*.xml" -o -name "*.toc" \) -delete
