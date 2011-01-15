# Installing python smart picker scripts
install( FILES FittingFunction.py InterestIO.py smart_picker.py
  PERMISSIONS GROUP_READ OWNER_READ
  DESTINATION libexec )
message( "smart_picker.py" )
install( FILES smart_picker.py
  PERMISSIONS GROUP_EXECUTE OWNER_EXECUTE OWNER_READ GROUP_READ
  DESTINATION bin )