{
  description = "A very basic flake that generates the DBCs by running a python script";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs =
    {
      self,
      nixpkgs,
    }:
    # Default for all nixpkgs
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];

      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f system);

      pkg_overlay = final: prev: {
        can_pkg = final.callPackage ./gen_tool/default.nix { };
      };
    in
    {
      overlays.default = pkg_overlay;

      packages = forAllSystems (
        system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ pkg_overlay ];
          };

        in
        {
          default = pkgs.can_pkg;
        }
      );

      # nix develop -c pytest gen_tool
      devShells = forAllSystems (
        system:
        let
          pkgs = import nixpkgs { inherit system; };
        in
        {
          default = pkgs.mkShell {
            packages = [ (pkgs.python313.withPackages (ps: [ ps.cantools ps.pytest ])) ];
          };
        }
      );
    };
}
