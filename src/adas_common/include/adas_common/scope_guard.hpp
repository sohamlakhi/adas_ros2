#ifndef SCOPE_GUARD_HPP_
#define SCOPE_GUARD_HPP_

template
<typename Func>
struct ScopeGuard
{
    ScopeGuard(Func func) : func{func} {}
    ~ScopeGuard() { func(); }
    Func func;
};

template
<typename Func>
ScopeGuard<Func> makeGuard(Func func)
{ return ScopeGuard<Func>{func}; }

#endif
