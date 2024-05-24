{ pkgs, python311Packages, fetchPypi, rev_hash }:

pkgs.stdenv.mkDerivation {
  name = "dbc_pkg";

  src = ./.;

  rev_string = rev_hash;

  buildInputs = [
    (python311Packages.cantools.overridePythonAttrs (old: {
      src = fetchPypi {
        pname = "cantools";
        version = "39.4.4";
        hash = "sha256-bo6Ri2ZxpiqfOZBUbs5WI+Hetx3vsc74WplVrDAdqZ4=";
      };
      doCheck = false;
    }))
  ]; # Python as a build dependency

  # Define the build phase to execute the scripts
  buildPhase = ''
    mkdir -p $out
    python json_to_dbc.py can_descriptor.json $out/car ${rev_hash} Orion RMS_PM Megasquirt
  '';
}
