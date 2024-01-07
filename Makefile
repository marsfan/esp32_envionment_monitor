lint:
	-docker container rm idf_linter
# Try both windows and linux commands
	-pwsh -C "rm -r -Force html_report"
	-rm -r html_report warnings.txt
	docker build -f Dockerfile.lint -t idftest . && docker run --name=idf_linter -it idftest
	docker cp idf_linter:/project/warnings.txt $(CURDIR)/warnings.txt
	docker cp idf_linter:/project/html_report $(CURDIR)/html_report
	docker cp idf_linter:/project/report.json $(CURDIR)/report.json