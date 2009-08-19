#include <pluginlib/class_list_macros.h>
#include <diagnostic_analyzer/diagnostic_analyzer.h>
#include <generic_analyzer/generic_analyzer.h>

REGISTER_CLASS(GenericAnalyzer, diagnostic_analyzer::GenericAnalyzer, diagnostic_analyzer::DiagnosticAnalyzer)
