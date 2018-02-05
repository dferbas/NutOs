This is a Nut/OS repository, which started with Nut/OS v5.0.5.

We fixed serious things at PPP implementation - proper process terminating,
where we modified at which context (thread) some actions are processed.
We also fixed a possibility to reenter a closed session and added a keep alive request sending
to keep a connection with no data transfer established.
Even such a connection is closed by some providers, e.g. O2.

We also added a NutUseCritical  macro,
which was finally the only way, how to deal with our GCC compiler for m68k.
Without this, there was a really hard issue at NutThreadResume,
which sometimes resulted in probably lost condition flags due to interrupts.
Construction

        NutEnterCritical();
        ...
        NutExitCritical();
        if (cnt) {
            NutEnterCritical();
            ...
            NutExitCritical();

was compiled for some code optimizations as below.
The problem is, if status word (condition flags) changes between point B and C.
These changes are lost, because they are overwritten at point D.
This was really very hard to find and after this was fixed, several "problems" were fixed.

I tried many ways, how to stay without NutUseCritical, but I was unable to find any.
>NutEnterCritical, point A

move.w    %sr, %d0
move.w    %d0, %d1
ori.l    #0x700, %d0
move.w    %d0, %sr

>NutExitCritical, point B
move.w    %d1, %sr

>NutEnterCritical, point C
move.w    %sr, %d0
move.w    %d0, %d2
ori.l    #0x700, %d0
move.w    %d0, %sr

>NutExitCritical, point D
move.w    %d1, %sr
