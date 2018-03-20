add_subdirectory_ifdef(
  CONFIG_HAS_MCUX
  mcux
  )
add_subdirectory_ifdef(
	SOC_FAMILY_LPC
	lpc
	)