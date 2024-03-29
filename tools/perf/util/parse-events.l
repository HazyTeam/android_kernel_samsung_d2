
%option reentrant
%option bison-bridge
%option prefix="parse_events_"
%option stack

%{
#include <errno.h>
#include "../perf.h"
#include "parse-events-bison.h"
#include "parse-events.h"

char *parse_events_get_text(yyscan_t yyscanner);
YYSTYPE *parse_events_get_lval(yyscan_t yyscanner);

static int __value(YYSTYPE *yylval, char *str, int base, int token)
{
	u64 num;

	errno = 0;
	num = strtoull(str, NULL, base);
	if (errno)
		return PE_ERROR;

	parse_events_lval.num = num;
	return token;
}

static int value(int base)
{
	return __value(parse_events_text, base, PE_VALUE);
}

static int raw(void)
{
	return __value(parse_events_text + 1, 16, PE_RAW);
}

static int sh_raw(void)
{
	return __value(parse_events_text + 2, 16, PE_SH_RAW);
}

static int fab_raw(void)
{
	return __value(parse_events_text + 2, 16, PE_FAB_RAW);
}


static int str(int token)
{
	parse_events_lval.str = strdup(parse_events_text);
	return token;
}

static int sym(int type, int config)
{
	parse_events_lval.num = (type << 16) + config;
	return PE_VALUE_SYM;
}

static int term(int type)
{
	parse_events_lval.num = type;
	return PE_TERM;
}

%}

%x mem
%s config
%x event

group		[^,{}/]*[{][^}]*[}][^,{}/]*
event_pmu	[^,{}/]+[/][^/]*[/][^,{}/]*
event		[^,{}/]+

num_dec		[0-9]+
num_hex		0x[a-fA-F0-9]+
num_raw_hex	[a-fA-F0-9]+
name		[a-zA-Z_*?][a-zA-Z\-0-9_*?]*
modifier_event	[ukhpGH]{1,8}
modifier_bp	[rwx]{1,3}

%%

%{
	{
		int start_token;

		start_token = parse_events_get_extra(yyscanner);

		if (start_token == PE_START_TERMS)
			BEGIN(config);
		else if (start_token == PE_START_EVENTS)
			BEGIN(event);

		if (start_token) {
			parse_events_set_extra(NULL, yyscanner);
			return start_token;
		}
         }
%}

<event>{

{group}		{
			BEGIN(INITIAL); yyless(0);
		}

{event_pmu}	|
{event}		{
			str(yyscanner, PE_EVENT_NAME);
			BEGIN(INITIAL); yyless(0);
			return PE_EVENT_NAME;
		}

.		|
<<EOF>>		{
			BEGIN(INITIAL); yyless(0);
		}

}

cpu-cycles|cycles				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES); }
stalled-cycles-frontend|idle-cycles-frontend	{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STALLED_CYCLES_FRONTEND); }
stalled-cycles-backend|idle-cycles-backend	{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_STALLED_CYCLES_BACKEND); }
instructions					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTRUCTIONS); }
cache-references				{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CACHE_REFERENCES); }
cache-misses					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CACHE_MISSES); }
branch-instructions|branches			{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_INSTRUCTIONS); }
branch-misses					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_MISSES); }
bus-cycles					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BUS_CYCLES); }
ref-cycles					{ return sym(yyscanner, PERF_TYPE_HARDWARE, PERF_COUNT_HW_REF_CPU_CYCLES); }
cpu-clock					{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CPU_CLOCK); }
task-clock					{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_TASK_CLOCK); }
page-faults|faults				{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS); }
minor-faults					{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS_MIN); }
major-faults					{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS_MAJ); }
context-switches|cs				{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CONTEXT_SWITCHES); }
cpu-migrations|migrations			{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CPU_MIGRATIONS); }
alignment-faults				{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_ALIGNMENT_FAULTS); }
emulation-faults				{ return sym(yyscanner, PERF_TYPE_SOFTWARE, PERF_COUNT_SW_EMULATION_FAULTS); }

L1-dcache|l1-d|l1d|L1-data		|
L1-icache|l1-i|l1i|L1-instruction	|
LLC|L2					|
dTLB|d-tlb|Data-TLB			|
iTLB|i-tlb|Instruction-TLB		|
branch|branches|bpu|btb|bpc		|
node					{ return str(yyscanner, PE_NAME_CACHE_TYPE); }

load|loads|read				|
store|stores|write			|
prefetch|prefetches			|
speculative-read|speculative-load	|
refs|Reference|ops|access		|
misses|miss				{ return str(yyscanner, PE_NAME_CACHE_OP_RESULT); }

	/*
	 * These are event config hardcoded term names to be specified
	 * within xxx/.../ syntax. So far we dont clash with other names,
	 * so we can put them here directly. In case the we have a conflict
	 * in future, this needs to go into '//' condition block.
	 */
config			{ return term(PARSE_EVENTS__TERM_TYPE_CONFIG); }
config1			{ return term(PARSE_EVENTS__TERM_TYPE_CONFIG1); }
config2			{ return term(PARSE_EVENTS__TERM_TYPE_CONFIG2); }
period			{ return term(PARSE_EVENTS__TERM_TYPE_SAMPLE_PERIOD); }
branch_type		{ return term(PARSE_EVENTS__TERM_TYPE_BRANCH_SAMPLE_TYPE); }

mem:			{ return PE_PREFIX_MEM; }
r{num_raw_hex}		{ return raw(); }
rs{num_raw_hex}		{ return sh_raw(); }
rm{num_raw_hex}		{ return fab_raw(); }
{num_dec}		{ return value(10); }
{num_hex}		{ return value(16); }

{modifier_event}	{ return str(PE_MODIFIER_EVENT); }
{modifier_bp}		{ return str(PE_MODIFIER_BP); }
{name}			{ return str(PE_NAME); }
"/"			{ return '/'; }
-			{ return '-'; }
,			{ return ','; }
:			{ return ':'; }
=			{ return '='; }

<mem>{
{modifier_bp}		{ return str(PE_MODIFIER_BP); }
:			{ return ':'; }
{num_dec}		{ return value(10); }
{num_hex}		{ return value(16); }
	/*
	 * We need to separate 'mem:' scanner part, in order to get specific
	 * modifier bits parsed out. Otherwise we would need to handle PE_NAME
	 * and we'd need to parse it manually. During the escape from <mem>
	 * state we need to put the escaping char back, so we dont miss it.
	 */
.			{ unput(*parse_events_text); BEGIN(INITIAL); }
	/*
	 * We destroy the scanner after reaching EOF,
	 * but anyway just to be sure get back to INIT state.
	 */
<<EOF>>			{ BEGIN(INITIAL); }
}

%%

int parse_events_wrap(void *scanner __maybe_unused)
{
	return 1;
}
