import { Suspense } from "react";
export const metadata = {
  title: "Sector",
};

export default async function Layout({ children }) {
  return (
    <>
      <Suspense fallback={<div>Loading...</div>}>{children}</Suspense>
    </>
  );
}
