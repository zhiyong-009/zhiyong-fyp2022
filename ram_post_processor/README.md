# How do I use the post processor?
The post processor class is provided as an example and might be incomplete or not adapted to your needs.

If you want to create your own post-processor from it:
- Create a package in which you define a class that derives from `PostProcessor`
- Override all the virtual member functions of `PostProcessor` in your new class
- Inside your new class functions append to the `program_` string your program instructions

You can also create a service and a Qt GUI arround the post processor.
