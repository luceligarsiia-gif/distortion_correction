# Derive compact ROM banks from the single original 129x256 gain table.
# Call before compilation if night_gain_q8.mem has been edited. No Python
# or other external runtime is required by the PDS batch build.
proc update_gain_file {path data} {
    if {[file exists $path]} {
        set handle [open $path r]
        set previous [read $handle]
        close $handle
        if {$previous eq $data} {return}
    }
    set handle [open $path w]
    fconfigure $handle -encoding ascii -translation lf
    puts -nonewline $handle $data
    close $handle
}
set gain_directory [file join [file dirname [info script]] source rtl img_process]
set gain_handle [open [file join $gain_directory night_gain_q8.mem] r]
set gain_words [regexp -all -inline {\S+} [read $gain_handle]]
close $gain_handle
if {[llength $gain_words] != 33024} {error "Gain table must contain 129 x 256 entries."}
foreach gain_word $gain_words {
    if {![regexp {^[0-9a-fA-F]{1,3}$} $gain_word]} {error "Invalid 12-bit gain table value: $gain_word"}
}
update_gain_file [file join $gain_directory night_gain_bulk_q8.mem] "[join [lrange $gain_words 0 32767] \n]\n"
update_gain_file [file join $gain_directory night_gain_last_q8.mem] "[join [lrange $gain_words 32768 end] \n]\n"
rename update_gain_file {}
unset gain_directory gain_handle gain_words gain_word
