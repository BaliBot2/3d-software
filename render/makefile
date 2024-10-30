RENDER_SCRIPT = render.py

render:
	@python3 $(RENDER_SCRIPT) $(JSON_FILE) $(OUTPUT_IMAGE)
	@if exist $(OUTPUT_IMAGE) ( \
		@echo "Visualization saved to $(OUTPUT_IMAGE)."; \
		explorer $(OUTPUT_IMAGE); \
	) else ( \
		@echo "Error: $(OUTPUT_IMAGE) not found."; \
		exit 1 \
	)
	@if exist "$(subst /,\\,$(JSON_FILE))" ( \
		@del /f /q "$(subst /,\\,$(JSON_FILE))" \
	) else ( \
		@echo "Error: $(JSON_FILE) not found."; \
		exit 1 \
	)

clean:
	del /f /q *.png
