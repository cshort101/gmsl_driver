#ifndef SAMPLES_COMMON_SAMPLEFRAMEWORK_HPP__
#define SAMPLES_COMMON_SAMPLEFRAMEWORK_HPP__

#include <Checks.hpp>

#include <WindowGLFW.hpp>
#ifdef VIBRANTE
#include <WindowEGL.hpp>
#else
#endif

#include <ProgramArguments.hpp>
#include <ConsoleColor.hpp>

#include <signal.h>
#include <cstring> // for memset
#include <iostream>

#include <dw/renderer/Renderer.h>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

extern void (*gUserKeyPressCallback)(int);
extern ProgramArguments gArguments;
extern WindowBase *gWindow;
extern bool gRun;

/// Specifies a 2D line segment from point a to point b, in floating point coordinates
typedef struct {
    dwVector2f a;
    dwVector2f b;
} dwLineSegment2Df;

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// signal handling
//void sig_int_handler(int sig);

// key press event
//void keyPressCallback(int key);

// draw line segment in 2D
// Required renderbuffer with the following properties:
// - Renderbuffer should be initialized with DW_RENDER_PRIM_LINELIST
// - Expected position format: DW_RENDER_FORMAT_R32G32_FLOAT
// - Expected position semantic: DW_RENDER_SEMANTIC_POS_XY
void drawLineSegments(const std::vector<dwLineSegment2Df> &segments,
                      dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);

// draw box
// Required renderbuffer with the following properties:
// - Renderbuffer should be initialized with DW_RENDER_PRIM_LINELIST
// - Expected position format: DW_RENDER_FORMAT_R32G32_FLOAT
// - Expected position semantic: DW_RENDER_SEMANTIC_POS_XY
void drawBoxes(const std::vector<dwBox2D> &boxes, const std::vector<uint32_t> *boxIds,
               float32_t normalizationWidth, float32_t normalizationHeight,
               dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);

// draw box with labels
// Required renderbuffer with the following properties:
// - Renderbuffer should be initialized with DW_RENDER_PRIM_LINELIST
// - Expected position format: DW_RENDER_FORMAT_R32G32_FLOAT
// - Expected position semantic: DW_RENDER_SEMANTIC_POS_XY
void drawBoxesWithLabels(const std::vector<std::pair<dwBox2D, std::string> > &boxesWithLabels,
                         float32_t normalizationWidth, float32_t normalizationHeight,
                         dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);


// init sample application
bool initSampleApp(int argc, const char **argv,
                   const ProgramArguments* arguments,
                   void (*userKeyPressCallback)(int),
                   uint32_t width, uint32_t height);

void releaseSampleApp();


#endif // SAMPLES_COMMON_SAMPLEFRAMEWORK_HPP__
