import { Suspense } from "react";
export const metadata = {
  title: "Chat",
  description: "Chat with the Research Bot",
};

export default async function Layout({ children }) {
  return (
    <>
      <Suspense fallback={<div>Loading...</div>}>{children}</Suspense>
    </>
  );
}
