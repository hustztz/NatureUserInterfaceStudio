#pragma once

#ifndef NUI_ERROR
#define NUI_ERROR(...) \
    fprintf(stderr, "# Error: NUI: >%s(%d): '%s()':\n", __FILE__, __LINE__, __FUNCTION__); \
    fprintf(stderr, __VA_ARGS__);
#endif

#ifdef _DEBUG
	#ifndef NUI_WARN
	#define NUI_WARN(...) \
		fprintf(stderr, "# Warning: NUI: >%s(%d): '%s()':\n", __FILE__, __LINE__, __FUNCTION__); \
		fprintf(stderr, __VA_ARGS__);
	#endif
#else
	#define NUI_WARN(...)	((void) 0)
#endif

#ifdef _DEBUG
#ifndef NUI_DEBUG
	#define NUI_DEBUG(...) \
		fprintf(stderr, "# Debug: NUI: "); \
		fprintf(stderr, __VA_ARGS__);
	#endif
#else
	#define NUI_DEBUG(...)	((void) 0)
#endif

#ifndef NUI_PRINT
#define NUI_PRINT(...) \
    fprintf(stderr, "# NUI: "); \
    fprintf(stderr, __VA_ARGS__);
#endif

