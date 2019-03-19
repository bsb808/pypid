# pypid

A general-purpose, simple Proportional-Integral-Derivative (PID) feedback control object.  Some of the features include...

 * Derivative in feedback or forward path
 * First or second-order low-pass filter on input signal
 * First or second-order low-pass filter on derivative signal
 * Inclusion of a rate sensor to directly measure derivative signal (instead of numerical estimate from input)
 * Unwrapping of inputs, e.g., for attitude control where input is an angle
 * Saturation/clipping of integral term for anti-wind

## Installation for Continued Development

 * cd ~/WorkingCopies
 * git clone git@github.com:bsb808/pypid.git
 * cd pypid
 * sudo python setup.py develop

## Documentation

Documentation is done using Sphinx.  See docs folder.
