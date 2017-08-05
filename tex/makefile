# Author: Javier Reyes

# Defines the name of the tex file
FILE=report

# Rule to update the pdf document based on the latex source and the bib reference file
$(FILE).pdf : $(FILE).tex chapters/sys_analysis.tex
	pdflatex $(FILE)
	bibtex $(FILE)
	pdflatex $(FILE)
	pdflatex $(FILE)

# Rule to clean the folder, deleting all the temporal files from the tex compilation
clean :
	find . \( -name "*-blx.bib" -o -name "*.vrb" -o -name "*.snm" -o -name "*.out" -o -name "*.nav" -o -name "*.aux" -o -name "*.bbl" -o -name "*.bcf" -o -name "*.blg" -o -name "*.lof" -o -name "*.log" -o -name "*.lot" -o -name "*.xml" -o -name "*.toc" \) -delete
