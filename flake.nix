{
  description = "A flake for KSU-MS's DBC for our DAQ repo";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-23.11";
  };

  outputs = { self, nixpkgs }:
  let 
    # Get the rev
    rev = if (self ? rev) then self.rev else "dirty";

    # Make an overlay to ref in other flakes
    dbc_overlay = final: prev: {
      can_pkg = final.callPackage ./default.nix { rev_hash = rev; };
    };

    # Abstract what platform we are building for
    forAllSystems = function:
      nixpkgs.lib.genAttrs [
        "x86_64-linux"
        "aarch64-linux"
      ] (system: function nixpkgs.legacyPackages.${system});

  in {
    # Export the overlay
    overlays.default = dbc_overlay;

    # Build for whatever systems in the forAllSystems
    packages = forAllSystems (pkgs: {
      default = pkgs.callPackage ./default.nix { rev_hash = rev; };
    });
  };
}
